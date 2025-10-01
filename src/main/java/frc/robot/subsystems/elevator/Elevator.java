package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  ElevatorIO io;

  protected final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer followerConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert leaderDisconnected;
  private final Alert followerDisconnected;
  private final Alert leaderTempFault;
  private final Alert followerTempFault;
  Trigger zeroTrigger;

  // ===== 2910-style state machine (names mirrored from Wrist) =====
  public enum WantedState {
    HOME, // drive to hardstop, zero to MIN_HEIGHT
    IDLE, // stop output
    MOVE_TO_POSITION, // closed-loop to a target height (m)
    HOLD, // hold current goal; useful for tuning
    MANUAL_VOLTAGE, // direct open-loop; useful for tuning
    OFF
  }

  private enum SystemState {
    HOMING,
    IDLING,
    MOVING_TO_POSITION,
    HOLDING,
    MANUAL_VOLTAGE,
    OFF
  }

  private WantedState wantedState = WantedState.OFF;
  private SystemState currentState = SystemState.OFF;

  // targets / params (state drives behavior)
  private double wantedHeight = ElevatorConstants.Preset.STOW; // meters
  private double holdHeight = ElevatorConstants.Preset.STOW; // meters
  private double manualVolts = 0.0;

  // homing params (parity with Wrist style)
  private static final double HOMING_PUSH_VOLTS = -4.0;
  private static final double HOMING_CURRENT_SPIKE_AMPS = 40.0;

  private boolean homed = false;

  public Elevator(ElevatorIO io) {
    this.io = io;

    leaderDisconnected =
        new Alert(getName() + " leader motor disconnected!", Alert.AlertType.kWarning);
    followerDisconnected =
        new Alert(getName() + " follower motor disconnected!", Alert.AlertType.kWarning);
    leaderTempFault = new Alert(getName() + " leader motor too hot! 🥵", Alert.AlertType.kWarning);
    followerTempFault =
        new Alert(getName() + " follower motor too hot! 🥵", Alert.AlertType.kWarning);

    zeroTrigger = new Trigger(() -> inputs.data.zeroSwitchTriggered());
    zeroTrigger.onFalse(
        new InstantCommand(
            () -> io.zeroElevatorPosition())); // TODO: Move away from command based and poll in
    // periodic...
  }

  // --- setWantedState overloads (identical pattern to Wrist) ---
  public void setWantedState(WantedState next) {
    this.wantedState = next;
  }

  /** For MOVE_TO_POSITION/HOLD with a meters payload. */
  public void setWantedState(WantedState next, double meters) {
    this.wantedHeight = clampToLimits(meters);
    setWantedState(next);
  }

  /** For MANUAL_VOLTAGE with a volts payload. */
  public void setWantedState(WantedState next, double volts, boolean isManual) {
    this.manualVolts = volts;
    setWantedState(next);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    leaderDisconnected.set(
        !motorConnectedDebouncer.calculate(getLeaderConnected()) && !Robot.isJITing());
    followerDisconnected.set(
        !followerConnectedDebouncer.calculate(getFollowerConnected()) && !Robot.isJITing());
    leaderTempFault.set(getLeaderTempFault());
    followerTempFault.set(getFollowerTempFault());

    // state machine
    currentState = handleStateTransition(wantedState);
    applyState(currentState);

    Logger.recordOutput("Elevator/WantedState", wantedState);
    Logger.recordOutput("Elevator/SystemState", currentState);
    Logger.recordOutput("Elevator/ReachedSetpoint", reachedSetpoint());
    Logger.recordOutput("Elevator/WantedHeightMeters", wantedHeight);
    Logger.recordOutput("Elevator/HoldHeightMeters", holdHeight);
    Logger.recordOutput("Elevator/Homed", homed);

    LoggedTracer.record("Elevator");
  }

  private SystemState handleStateTransition(WantedState wanted) {
    if (!getLeaderConnected()) return SystemState.OFF;

    switch (wanted) {
      case HOME:
        homed = false; // re-home when asked
        return SystemState.HOMING;

      case IDLE:
        return SystemState.IDLING;

      case MOVE_TO_POSITION:
        return homed ? SystemState.MOVING_TO_POSITION : SystemState.HOMING;

      case HOLD:
        if (currentState != SystemState.HOLDING) {
          holdHeight = clampToLimits(holdHeight);
        }
        return homed ? SystemState.HOLDING : SystemState.HOMING;

      case MANUAL_VOLTAGE:
        return SystemState.MANUAL_VOLTAGE;

      case OFF:
      default:
        return SystemState.OFF;
    }
  }

  private void applyState(SystemState state) {
    double volts = 0.0;
    double commandedMeters = holdHeight;

    switch (state) {
      case HOMING -> {
        // push gently into hardstop; detect current spike to zero
        io.elevatorMax(); // align controller to known side before pushing (optional hook)
        io.setElevatorVoltage(HOMING_PUSH_VOLTS);

        boolean torqueSpike = Math.abs(getTorqueCurrentAmps()) > HOMING_CURRENT_SPIKE_AMPS;
        boolean zeroSwitchHit = inputs.data.zeroSwitchTriggered();
        if (torqueSpike || zeroSwitchHit) {
          io.zeroElevatorPosition(); // set encoder to MIN height
          homed = true;
          setWantedState(WantedState.IDLE);
          holdHeight = ElevatorConstants.Preset.STOW;
        }
      }

      case IDLING -> {
        volts = 0.0;
        io.setElevatorVoltage(volts);
      }

      case MOVING_TO_POSITION -> {
        commandedMeters = clampToLimits(wantedHeight);
        holdHeight = commandedMeters; // keep HOLD in sync for quick flip to HOLD
        io.setElevatorPosition(commandedMeters);
      }

      case HOLDING -> {
        commandedMeters = clampToLimits(holdHeight);
        io.setElevatorPosition(commandedMeters);
      }

      case MANUAL_VOLTAGE -> {
        volts = manualVolts;
        io.setElevatorVoltage(volts);
      }

      case OFF -> {
        volts = 0.0;
        io.setElevatorVoltage(volts);
      }
    }

    // end-of-apply guard (parity with Wrist)
    if (!getLeaderConnected()) io.setElevatorVoltage(0.0);

    // telemetry parity
    Logger.recordOutput("Elevator/CommandVolts", volts);
    Logger.recordOutput("Elevator/CommandHeightMeters", commandedMeters);
  }

  // ===== helpers (same placement as Wrist helpers) =====

  /** Clamp requested height to software limits. */
  private static double clampToLimits(double meters) {
    return MathUtil.clamp(
        meters, ElevatorConstants.MIN_HEIGHT_METERS, ElevatorConstants.MAX_HEIGHT_METERS);
  }

  /** Whether the elevator is within tolerance of the wanted height. */
  public boolean reachedSetpoint() {
    return Math.abs(getPositionMeters() - wantedHeight) <= ElevatorConstants.GOAL_TOLERANCE_M;
  }

  // IOData Getters (names/shape mirror Wrist accessors)
  public double getPositionMeters() {
    return inputs.data.positionMeters();
  }

  public double getVelocityMetersPerSec() {
    return inputs.data.velocityMetersPerSec();
  }

  public double getAppliedVoltage() {
    return inputs.data.appliedVolts();
  }

  public double getSupplyCurrentAmps() {
    return inputs.data.supplyCurrentAmps();
  }

  public double getTorqueCurrentAmps() {
    return inputs.data.torqueCurrentAmps();
  }

  public double getLeaderTempCelsius() {
    return inputs.data.tempCelsiusLeader();
  }

  public double getFollowerTempCelsius() {
    return inputs.data.tempCelsiusFollower();
  }

  public boolean getLeaderConnected() {
    return inputs.data.motorConnected();
  }

  public boolean getFollowerConnected() {
    return inputs.data.followerConnected();
  }

  public boolean getLeaderTempFault() {
    return inputs.data.tempFault();
  }

  public boolean getFollowerTempFault() {
    return inputs.data.followerTempFault();
  }
}
