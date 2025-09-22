package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  WristIO io;

  protected final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, DebounceType.kFalling);
  private final Alert disconnected;
  private final Alert tempFault;

  // ===== 2910-style state machine =====
  public enum WantedState {
    HOME,              // drive to hardstop, zero to MIN_ANGLE
    IDLE,              // stop output
    MOVE_TO_POSITION,  // closed-loop to a target angle
    HOLD,              // hold current goal; useful for tuning
    MANUAL_VOLTAGE,    // direct open-loop; useful for tuning
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
  private Rotation2d wantedAngle = WristConstants.Preset.STOW; // target for MOVE_TO_POSITION
  private Rotation2d holdAngle   = WristConstants.Preset.STOW; // sticky goal for HOLD
  private double manualVolts = 0.0;

  // homing params (match your original intent)
  private static final double HOMING_PUSH_VOLTS = -6.0;
  private static final double HOMING_CURRENT_SPIKE_AMPS = 40.0;

  private boolean homed = false;

  public Wrist(WristIO io) {
    this.io = io;

    disconnected = new Alert(getName() + " motor disconnected!", Alert.AlertType.kWarning);
    tempFault = new Alert(getName() + " motor too hot! 🥵", Alert.AlertType.kWarning);
  }

  // --- setWantedState overloads (all funnel to the “super” one) ---
  public void setWantedState(WantedState next) {
    this.wantedState = next;
  }

  /** For MOVE_TO_POSITION/HOLD with an angle payload. */
  public void setWantedState(WantedState next, Rotation2d angle) {
    this.wantedAngle = clampToLimits(angle);
    setWantedState(next);
  }

  /** For MANUAL_VOLTAGE with a volts payload. */
  public void setWantedState(WantedState next, double volts) {
    this.manualVolts = volts;
    setWantedState(next);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);

    disconnected.set(!motorConnectedDebouncer.calculate(getConnected()) && !Robot.isJITing());
    tempFault.set(getTempFault());
    // state machine
    currentState = handleStateTransition(wantedState);
    applyState(currentState);

    Logger.recordOutput("Wrist/WantedState", wantedState);
    Logger.recordOutput("Wrist/SystemState", currentState);
    Logger.recordOutput("Wrist/ReachedSetpoint", reachedSetpoint());
    Logger.recordOutput("Wrist/WantedAngle", wantedAngle);
    Logger.recordOutput("Wrist/HoldAngle", holdAngle);
    Logger.recordOutput("Wrist/Homed", homed);

    LoggedTracer.record("Wrist");
  }

  private SystemState handleStateTransition(WantedState wanted) {
    if (!getConnected()) return SystemState.OFF;

    switch (wanted) {
      case HOME:
        homed = false; // re-home when asked
        return SystemState.HOMING;

      case IDLE:
        return SystemState.IDLING;

      case MOVE_TO_POSITION:
        return homed ? SystemState.MOVING_TO_POSITION : SystemState.HOMING;

      case HOLD:
        // capture/keep a sticky hold target when entering HOLD
        if (currentState != SystemState.HOLDING) {
          holdAngle = clampToLimits(holdAngle);
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
    Rotation2d commandedAngle = holdAngle;

    switch (state) {
      case HOMING -> {
        // push gently into hardstop; detect current spike to zero
        io.wristMax(); // align controller to known side before pushing
        io.setWristVoltage(HOMING_PUSH_VOLTS);

        boolean torqueSpike = Math.abs(getTorqueCurrentAmps()) > HOMING_CURRENT_SPIKE_AMPS;
        if (torqueSpike) {
          io.zeroWristPosition(); // set encoder to MIN_ANGLE
          homed = true;
          // after zeroing, default to IDLE; callers can request a move next
          setWantedState(WantedState.IDLE);
          holdAngle = WristConstants.Preset.STOW;
        }
      }

      case IDLING -> {
        volts = 0.0;
        io.setWristVoltage(volts);
      }

      case MOVING_TO_POSITION -> {
        commandedAngle = clampToLimits(wantedAngle);
        holdAngle = commandedAngle; // keep HOLD in sync for quick flip to HOLD
        io.setWristPosition(commandedAngle);
      }

      case HOLDING -> {
        commandedAngle = clampToLimits(holdAngle);
        io.setWristPosition(commandedAngle);
      }

      case MANUAL_VOLTAGE -> {
        volts = manualVolts;
        io.setWristVoltage(volts);
      }

      case OFF -> {
        volts = 0.0;
        io.setWristVoltage(volts);
      }
    }

    // end-of-apply guard (parity with EndEffector)
    if (!getConnected()) io.setWristVoltage(0.0);

    // telemetry parity
    Logger.recordOutput("Wrist/CommandVolts", volts);
    Logger.recordOutput("Wrist/CommandAngleRad", commandedAngle.getRadians());
  }

  // ===== helpers (kept at end, like coralArmed() etc.) =====

  /** Clamp any requested angle to software limits. */
  private static Rotation2d clampToLimits(Rotation2d angle) {
    double rot = MathUtil.clamp(
        angle.getRotations(),
        WristConstants.MIN_ANGLE.getRotations(),
        WristConstants.MAX_ANGLE.getRotations());
    return Rotation2d.fromRotations(rot);
  }

  /** Whether the wrist is within tolerance of the wanted angle (simple threshold, like 2910). */
  public boolean reachedSetpoint() {
    return Math.abs(getPositionRads() - wantedAngle.getRadians())
        <= WristConstants.GOAL_TOLERANCE_RAD;
  }

  // IOData Getters (names/shape mirror EndEffector)
  public double getPositionRads() { return inputs.data.positionRad(); }
  public double getVelocityRadsPerSec() { return inputs.data.velocityRadPerSec(); }
  public double getAppliedVoltage() { return inputs.data.appliedVolts(); }
  public double getSupplyCurrentAmps() { return inputs.data.supplyCurrentAmps(); }
  public double getTorqueCurrentAmps() { return inputs.data.torqueCurrentAmps(); }
  public double getTempCelsius() { return inputs.data.tempCelsius(); }
  public boolean getConnected() { return inputs.data.motorConnected(); }
  public boolean getTempFault() { return inputs.data.tempFault(); }

}
