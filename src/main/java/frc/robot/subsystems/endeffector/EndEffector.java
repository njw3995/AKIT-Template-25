package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  EndEffectorIO io;

  protected final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;
  private final Alert tempFault;

  private final Debouncer algaeDebouncer = new Debouncer(1.0, Debouncer.DebounceType.kRising);
  private final LinearFilter currentFilter =
      LinearFilter.movingAverage(EndEffectorConstants.CURRENT_FILTER_SAMPLE_SIZE);
  private final LinearFilter velocityFilter =
      LinearFilter.movingAverage(EndEffectorConstants.VELOCITY_FILTER_SAMPLE_SIZE);

  // ===== 2910-style state machine =====
  public enum WantedState {
    COLLECT_ALGAE,
    HOLD_ALGAE,
    HOLD_ALGAE_HARDER,
    EJECT_ALGAE,
    EJECT_ALGAE_PROCESSOR,
    COLLECT_CORAL,
    HOLD_CORAL,
    EJECT_CORAL_L1,
    EJECT_CORAL_FORWARD,
    OFF
  }

  private enum SystemState {
    COLLECTING_ALGAE,
    HOLDING_ALGAE,
    HOLDING_ALGAE_HARDER,
    EJECTING_ALGAE,
    EJECTING_ALGAE_PROCESSOR,
    COLLECTING_CORAL,
    HOLDING_CORAL,
    EJECTING_CORAL_L1,
    EJECTING_CORAL,
    INDEXING_CORAL_FORWARD,
    INDEXING_CORAL_BACKWARD,
    OFF
  }

  private WantedState wantedState = WantedState.OFF;
  private SystemState currentState = SystemState.OFF;

  private boolean hasAlgaeDebounced = false;

  public EndEffector(EndEffectorIO io) {
    this.io = io;

    disconnected = new Alert(getName() + " motor disconnected!", Alert.AlertType.kWarning);
    tempFault = new Alert(getName() + " motor too hot! 🥵", Alert.AlertType.kWarning);
  }

  public void setWantedState(WantedState next) {
    this.wantedState = next;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);

    disconnected.set(!motorConnectedDebouncer.calculate(getConnected()) && !Robot.isJITing());
    tempFault.set(inputs.data.tempFault());

    boolean algaeRaw = computeAlgaeRaw();
    hasAlgaeDebounced = algaeDebouncer.calculate(algaeRaw);

    // state machine
    currentState = handleStateTransition(wantedState);
    applyState(currentState);

    // logs
    Logger.recordOutput("EndEffector/WantedState", wantedState);
    Logger.recordOutput("EndEffector/SystemState", currentState);
    Logger.recordOutput("EndEffector/HasAlgae", hasAlgae());
    Logger.recordOutput("EndEffector/HasCoral", hasCoral());
    Logger.recordOutput("EndEffector/CoralArmed", coralArmed());
    Logger.recordOutput("EndEffector/Connected", getConnected());
    Logger.recordOutput("EndEffector/TempFault", getTempFault());

    LoggedTracer.record("EndEffector");
  }

  private SystemState handleStateTransition(WantedState wantedState) {
    if (getTempFault() || !getConnected()) return SystemState.OFF;

    switch (wantedState) {
      case COLLECT_ALGAE:
        if (hasAlgae()) return SystemState.HOLDING_ALGAE_HARDER;
        return SystemState.COLLECTING_ALGAE;

      case HOLD_ALGAE:
        return SystemState.HOLDING_ALGAE;

      case HOLD_ALGAE_HARDER:
        return SystemState.HOLDING_ALGAE_HARDER;

      case EJECT_ALGAE:
        return SystemState.EJECTING_ALGAE;

      case EJECT_ALGAE_PROCESSOR:
        return SystemState.EJECTING_ALGAE_PROCESSOR;

        // --- Coral auto index state machine (internal) ---
      case COLLECT_CORAL:
        { // TODO: Jam / Sensor-Miss Watchdogs for coral indexing
          // Sequence goal:
          //   COLLECTING (fast in) -> INDEX_BACKWARD (arm: both beams) -> INDEX_FORWARD (settle to
          // indexed) -> HOLD
          // All transitions are sensor-driven and multi-cycle safe.

          switch (currentState) {
            case HOLDING_CORAL:
              // Re-index or recollect based on where the piece actually is now
              if (!hasCoral()) return SystemState.COLLECTING_CORAL; // fell out → start over
              if (coralArmed())
                return SystemState.INDEXING_CORAL_FORWARD; // both beams → index forwards
              if (coralIndexed())
                return SystemState.INDEXING_CORAL_BACKWARD; // placement-only → index backwards
              /* else */ return SystemState.COLLECTING_CORAL; // intake-only → keep collecting

              // Collect until not seen by the intake beam break && seen by the placement beam break
            case COLLECTING_CORAL:
              if (!hasCoral())
                return SystemState.COLLECTING_CORAL; // still nothing → keep collecting
              if (coralIndexed())
                return SystemState.INDEXING_CORAL_BACKWARD; // past intake only → index backwards
              /* else */ return SystemState.COLLECTING_CORAL; // keep collecting

              // Index backwards until seen by the intake beam break && seen by the placement beam
              // break
            case INDEXING_CORAL_BACKWARD:
              if (!hasCoral()) return SystemState.COLLECTING_CORAL; // lost it → re-collect
              if (coralArmed())
                return SystemState.INDEXING_CORAL_FORWARD; // indexed backwards → index forwards
              /* else */ return SystemState.INDEXING_CORAL_BACKWARD; // keep indexing backwards

              // Index forwards until not seen by the intake beam break && seen by the placement
              // beam break
            case INDEXING_CORAL_FORWARD:
              if (!hasCoral()) return SystemState.COLLECTING_CORAL; // lost it → re-collect
              if (coralIndexed()) return SystemState.HOLDING_CORAL; // coral properly indexed
              /* else */ return SystemState.INDEXING_CORAL_FORWARD; // keep indexing forwards

              // Start / unexpected: choose the right phase from sensors
            default:
              if (!hasCoral()) return SystemState.COLLECTING_CORAL; // nothing → start collecting
              if (coralArmed())
                return SystemState.INDEXING_CORAL_FORWARD; // both beams → index forwards
              if (coralIndexed())
                return SystemState.INDEXING_CORAL_BACKWARD; // placement-only → index backwards
              /* else */ return SystemState.COLLECTING_CORAL; // intake-only → keep collecting
          }
        }

      case HOLD_CORAL:
        return SystemState
            .HOLDING_CORAL; // TODO: If drivers want it to auto index without them doing anything,
        // add in state machine logic here.

      case EJECT_CORAL_L1:
        return SystemState.EJECTING_CORAL_L1;

      case EJECT_CORAL_FORWARD:
        return SystemState.EJECTING_CORAL;

      case OFF:
      default:
        return SystemState.OFF;
    }
  }

  private void applyState(SystemState currentState) {
    double volts = 0.0;

    switch (currentState) {
        // Algae
      case COLLECTING_ALGAE -> volts = EndEffectorConstants.CollectingVoltages.ALGAE;
      case HOLDING_ALGAE -> volts = EndEffectorConstants.HoldingVoltages.ALGAE;
      case HOLDING_ALGAE_HARDER -> volts = EndEffectorConstants.HoldingVoltages.ALGAE_HARDER;
      case EJECTING_ALGAE -> volts = EndEffectorConstants.EjectingVoltages.ALGAE;
      case EJECTING_ALGAE_PROCESSOR -> volts =
          EndEffectorConstants.EjectingVoltages.ALGAE_PROCESSOR;

        // Coral
      case COLLECTING_CORAL -> volts = EndEffectorConstants.CollectingVoltages.CORAL;
      case HOLDING_CORAL -> volts = EndEffectorConstants.HoldingVoltages.CORAL;
      case INDEXING_CORAL_FORWARD -> volts = EndEffectorConstants.IndexingVoltages.CORAL_FORWARD;
      case INDEXING_CORAL_BACKWARD -> volts = EndEffectorConstants.IndexingVoltages.CORAL_BACKWARD;
      case EJECTING_CORAL_L1 -> volts = EndEffectorConstants.EjectingVoltages.CORAL_L1;
      case EJECTING_CORAL -> volts = EndEffectorConstants.EjectingVoltages.CORAL;

      case OFF -> volts = 0.0;
    }

    if (getTempFault() || !getConnected()) volts = 0.0;

    io.runVolts(volts);

    Logger.recordOutput("EndEffector/CommandVolts", volts);
  }

  // ===== helpers (kept in your style) =====

  /** Raw algae detection (filtered current & velocity), no debouncing. */
  private boolean
      computeAlgaeRaw() { // TODO: Double check that there are no false positives / false negatives
    // on falling edges...
    if (Constants.getMode() == Constants.Mode.SIM) return false; // change if you want
    double filteredCurrent = currentFilter.calculate(getTorqueCurrentAmps());
    double filteredVelocityAbs = Math.abs(velocityFilter.calculate(getVelocityRadsPerSec()));
    return filteredCurrent > EndEffectorConstants.HAS_ALGAE_CURRENT_AMPS
        && filteredVelocityAbs < EndEffectorConstants.HAS_ALGAE_MAX_VELOCITY_RAD_PER_SEC;
  }

  public boolean hasAlgae() {
    return hasAlgaeDebounced;
  }

  public boolean hasCoral() {
    return getIntakeBeamBreakDetected() || getPlacementBeamBreakDetected();
  }

  private boolean coralArmed() {
    return getIntakeBeamBreakDetected() && getPlacementBeamBreakDetected();
  }

  private boolean coralIndexed() {
    return !getIntakeBeamBreakDetected() && getPlacementBeamBreakDetected();
  }

  // IOData Getters
  public double getPositionRads() {
    return inputs.data.positionRads();
  }

  public double getVelocityRadsPerSec() {
    return inputs.data.velocityRadsPerSec();
  }

  public double getAppliedVoltage() {
    return inputs.data.appliedVoltage();
  }

  public double getSupplyCurrentAmps() {
    return inputs.data.supplyCurrentAmps();
  }

  public double getTorqueCurrentAmps() {
    return inputs.data.torqueCurrentAmps();
  }

  public double getTempCelsius() {
    return inputs.data.tempCelsius();
  }

  public boolean getTempFault() {
    return inputs.data.tempFault();
  }

  public boolean getIntakeBeamBreakDetected() {
    return inputs.data.intakeBeamBreakDetected();
  }

  public boolean getPlacementBeamBreakDetected() {
    return inputs.data.placementBeamBreakDetected();
  }

  public boolean getConnected() {
    return inputs.data.connected();
  }
}
