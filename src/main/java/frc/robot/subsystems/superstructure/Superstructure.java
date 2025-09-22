package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.endeffector.EndEffector;

import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.SuperstructureState.CurrentState;
import frc.robot.subsystems.superstructure.SuperstructureState.Mode;
import frc.robot.subsystems.superstructure.SuperstructureState.WantedState;
import frc.robot.subsystems.superstructure.SuperstructurePose.Pose;
import frc.robot.subsystems.superstructure.SuperstructurePose.ElevatorLevel;
import frc.robot.subsystems.superstructure.SuperstructurePose.WristPreset;

/**
 * 2910-style coordinator that owns high-level superstructure behavior.
 * - periodic() -> handleStateTransitions() -> applyStates()
 * - private action methods (only Superstructure calls them)
 * - no device-specific constants in here; all setpoints live in SuperstructurePose
 */
public class Superstructure extends SubsystemBase {
  // Subsystems
  private final Elevator elevator;
  private final Wrist wrist;
  private final EndEffector effector;

  // State machine
  private WantedState wantedState = WantedState.DEFAULT;
  private CurrentState currentState = CurrentState.STOPPED;
  private CurrentState previousState = CurrentState.STOPPED;

  private Mode mode = Mode.CORAL;
  private ElevatorLevel coralLevel = ElevatorLevel.L1; // default

  private boolean needWristStowFirst = false;
  private boolean wristStowedForPose = false;
  private boolean elevatorAtPose     = false;
  private Pose lastPose = SuperstructurePose.STOW;

  // One-shot HOME dispatch
  private boolean homeIssuedThisEntry = false;

  public Superstructure(Elevator elevator, Wrist wrist, EndEffector effector) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.effector = effector;
  }

  // ===== External API =====
  public void setWantedState(WantedState state) { wantedState = state; }
  public CurrentState getCurrentState() { return currentState; }

  /** Button-friendly setter. */
  public Command setStateCommand(WantedState state) {
    return new InstantCommand(() -> setWantedState(state));
  }

  // ===== Scheduler entry =====
  @Override
  public void periodic() {
    currentState = handleStateTransitions();
    applyStates();
    Logger.recordOutput("Superstructure/WantedState", wantedState);
    Logger.recordOutput("Superstructure/CurrentState", currentState);
    Logger.recordOutput("Superstructure/PreviousState", previousState);
  }

  // ===== Transitions (Wanted -> Current) =====
  private CurrentState handleStateTransitions() {
    previousState = currentState;

    switch (wantedState) {
      case STOP  -> currentState = CurrentState.STOPPED;
      case HOME  -> currentState = CurrentState.HOMING;
      case STOW  -> currentState = CurrentState.STOWING;

      case DEFAULT -> {
        if (effector.hasCoral())      currentState = CurrentState.HOLDING_CORAL;
        else if (effector.hasAlgae()) currentState = CurrentState.HOLDING_ALGAE;
        else                          currentState = CurrentState.NO_PIECE;
      }

      // Coral
      case INTAKE_CORAL -> currentState = CurrentState.INTAKING_CORAL;
      case HOLD_CORAL   -> currentState = CurrentState.HOLDING_CORAL;
      case SCORE_L1     -> currentState = CurrentState.SCORING_L1;
      case SCORE_L2     -> currentState = CurrentState.SCORING_L2;
      case SCORE_L3     -> currentState = CurrentState.SCORING_L3;
      case SCORE_L4     -> currentState = CurrentState.SCORING_L4;

      // Algae
      case INTAKE_ALGAE_GROUND   -> currentState = CurrentState.INTAKING_ALGAE_GROUND;
      case INTAKE_ALGAE_REEF     -> currentState = CurrentState.INTAKING_ALGAE_REEF;
      case HOLD_ALGAE            -> currentState = CurrentState.HOLDING_ALGAE;
      case SCORE_ALGAE_PROCESSOR -> currentState = CurrentState.SCORING_ALGAE_PROCESSOR;

      default -> currentState = CurrentState.STOPPED;
    }

    if (currentState != previousState) {
      if (currentState == CurrentState.HOMING) homeIssuedThisEntry = false;
    }

    return currentState;
  }

  // ===== Apply (Current -> private actions) =====
  private void applyStates() {
    if (previousState != currentState) {
        switch (currentState) {
          case SCORING_L1, SCORING_L2, SCORING_L3, SCORING_L4,
               INTAKING_ALGAE_REEF, INTAKING_ALGAE_GROUND:
               resetPoseGates();
            break;
          default:
            break;
        }
      }
    switch (currentState) {
      case STOPPED -> doStopped();
      case HOMING  -> doHome();
      case STOWING -> doStow();
      case NO_PIECE-> doNoPiece();

      // Coral
      case INTAKING_CORAL -> doIntakeCoral();
      case HOLDING_CORAL  -> doHoldCoral();
      case SCORING_L1     -> doScoreCoralL1();
      case SCORING_L2     -> doScoreCoral(ElevatorLevel.L2);
      case SCORING_L3     -> doScoreCoral(ElevatorLevel.L3);
      case SCORING_L4     -> doScoreCoralL4();

      // Algae
      case INTAKING_ALGAE_GROUND   -> doIntakeAlgaeGround();
      case INTAKING_ALGAE_REEF     -> doIntakeAlgaeReef();
      case HOLDING_ALGAE           -> doHoldAlgae();
      case SCORING_ALGAE_PROCESSOR -> doScoreAlgaeProcessor();
    }
  }

  // ===== Private actions (only Superstructure can call) =====

  private void doStopped() {
    elevator.setWantedState(Elevator.WantedState.IDLE);
    wrist.setWantedState(Wrist.WantedState.IDLE);
    effector.setWantedState(EndEffector.WantedState.OFF);
  }

  private void doHome() { //TODO: Should have boolean accessor if homed or not for subsystems
    // Issue HOME once on entry; subsystems run their own homing routine and
    // flip themselves to IDLE when done.
    if (!homeIssuedThisEntry) {
      elevator.setWantedState(Elevator.WantedState.HOME);
      wrist.setWantedState(Wrist.WantedState.HOME);
      homeIssuedThisEntry = true;
    }

    // See above Consider "homed" once both are at their STOW setpoints (close enough).
    boolean elevatorAtStow = Math.abs(elevator.getPositionMeters() - ElevatorConstants.Preset.STOW)
        <= ElevatorConstants.GOAL_TOLERANCE_M;
    boolean wristAtStow = Math.abs(wrist.getPositionRads() - WristConstants.Preset.STOW.getRadians())
        <= WristConstants.GOAL_TOLERANCE_RAD;

    if (elevatorAtStow && wristAtStow) {
      setWantedState(WantedState.DEFAULT);
    }
  }

  private void doStow() {
    goToPose(SuperstructurePose.STOW);
  }

  private void doNoPiece() {
    goToPose(SuperstructurePose.STOW);
    effector.setWantedState(EndEffector.WantedState.OFF);
  }

  // --- Coral ---
  private void doIntakeCoral() {
    effector.setWantedState(EndEffector.WantedState.COLLECT_CORAL);
  }

  private void doHoldCoral() {
    effector.setWantedState(EndEffector.WantedState.HOLD_CORAL);
  }

  private void doScoreCoralL1() {
    goToPose(SuperstructurePose.coralDunk(ElevatorLevel.L1));
    if (isReadyToScore()) {
      effector.setWantedState(EndEffector.WantedState.EJECT_CORAL_L1);
      if (!effector.hasCoral()) setWantedState(WantedState.DEFAULT);
    } else {
      effector.setWantedState(EndEffector.WantedState.HOLD_CORAL);
    }
  }

  private void doScoreCoral(ElevatorLevel level) {
    goToPose(SuperstructurePose.coralDunk(level));
    if (isReadyToScore()) {
      effector.setWantedState(EndEffector.WantedState.EJECT_CORAL_FORWARD);
      if (!effector.hasCoral()) setWantedState(WantedState.DEFAULT);
    } else {
      effector.setWantedState(EndEffector.WantedState.HOLD_CORAL);
    }
  }

  private void doScoreCoralL4() {
    // Use auto hover in auto, regular in teleop (matches your Wrist preset set)
    var pose = DriverStation.isAutonomous()
        ? SuperstructurePose.coralHover(ElevatorLevel.L4, true)
        : SuperstructurePose.coralDunk(ElevatorLevel.L4);
    goToPose(pose);

    if (isReadyToScore()) {
      effector.setWantedState(EndEffector.WantedState.EJECT_CORAL_FORWARD);
      if (!effector.hasCoral()) setWantedState(WantedState.DEFAULT);
    } else {
      effector.setWantedState(EndEffector.WantedState.HOLD_CORAL);
    }
  }

  // --- Algae ---
  private void doIntakeAlgaeGround() {
    goToPose(SuperstructurePose.ALGAE_GROUND_INTAKE);
    effector.setWantedState(EndEffector.WantedState.COLLECT_ALGAE);
    if (effector.hasAlgae()) setWantedState(WantedState.HOLD_ALGAE);
  }

  private void doIntakeAlgaeReef() {
    // TODO: Tie in RobotState/FieldConstants
    goToPose(SuperstructurePose.ALGAE_REEF_LOW_INTAKE);
    effector.setWantedState(EndEffector.WantedState.COLLECT_ALGAE);
    if (effector.hasAlgae()) setWantedState(WantedState.HOLD_ALGAE);
  }

  private void doHoldAlgae() {
    goToPose(SuperstructurePose.ALGAE_TRAVEL_HOLD);
    effector.setWantedState(EndEffector.WantedState.HOLD_ALGAE);
  }

  private void doScoreAlgaeProcessor() {
    goToPose(SuperstructurePose.ALGAE_AT_PROCESSOR);
    if (isReadyToScore()) {
      effector.setWantedState(EndEffector.WantedState.EJECT_ALGAE_PROCESSOR);

    } else {
      effector.setWantedState(EndEffector.WantedState.HOLD_ALGAE);
    }
  }

  // ===== Pose applier (pose = elevator meters + wrist angle only) =====
  private void goToPose(Pose pose) {
    if (!samePose(pose, lastPose)) {
      resetPoseGates();
      lastPose = pose;
    }
    // wrist → elevator → wrist (unchanged)
    if (needWristStowFirst && !wristStowedForPose) {
      wrist.setWantedState(Wrist.WantedState.MOVE_TO_POSITION, WristConstants.Preset.STOW);
      if (wrist.reachedSetpoint()) { wristStowedForPose = true; needWristStowFirst = false; }
      else return;
    }
    if (wristStowedForPose && !elevatorAtPose) {
      elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, pose.elevator().meters);
      if (elevator.reachedSetpoint()) { elevatorAtPose = true; }
      else return;
    }
    wrist.setWantedState(Wrist.WantedState.MOVE_TO_POSITION, pose.wrist().angle);
  }

  // ===== Helpers =====
  private boolean isReadyToScore() { //TODO: Add check for pose via RobotState.
    return elevator.reachedSetpoint() && wrist.reachedSetpoint();
  }

  private static boolean samePose(Pose a, Pose b) {
    if (a == b) return true;
    if (a == null || b == null) return false;
    // compare values (factories may create new instances)
    return a.elevator().meters == b.elevator().meters
        && Math.abs(a.wrist().angle.getRadians() - b.wrist().angle.getRadians()) < 1e-9;
  }
  
  private void resetPoseGates() {
    needWristStowFirst = true;
    wristStowedForPose = false;
    elevatorAtPose     = false;
  }

  public void toggleMode() {
    mode = (mode == Mode.CORAL) ? Mode.ALGAE : Mode.CORAL;
  }
  
  public void setCoralLevel(ElevatorLevel level) {
    coralLevel = level;
    Logger.recordOutput("Superstructure/CoralLevel", coralLevel);
  }
  
  // deploy uses your existing states & goToPose gates
  public void requestDeploy() {
    if (mode == Mode.CORAL) {
      switch (coralLevel) {
        case L1 -> setWantedState(WantedState.SCORE_L1);
        case L2 -> setWantedState(WantedState.SCORE_L2);
        case L3 -> setWantedState(WantedState.SCORE_L3);
        case L4 -> setWantedState(WantedState.SCORE_L4);
        default -> setWantedState(WantedState.STOW);
      }
    } else { // ALGAE
      setWantedState(WantedState.INTAKE_ALGAE_REEF);
    }
  }

  public Command toggleModeCommand() {
    return new InstantCommand(this::toggleMode);
  }
  
  public Command wristToggleCommand() {
    return new InstantCommand(this::toggleWristInternal); // your private helper already exists
  }
  
  public Command setDeployedCoralLevelCommand(SuperstructurePose.ElevatorLevel lvl) {
    return new InstantCommand(() -> setCoralLevel(lvl));
  }
  
  public Command deployCommand() {
    return new InstantCommand(this::requestDeploy);
  }
  
  public Command stowCommand() {
    return setStateCommand(SuperstructureState.WantedState.STOW);
  }
  
  // Intake (hold)
  public Command intakeStartCommand() {
    return new InstantCommand(() -> {
      if (mode == SuperstructureState.Mode.CORAL) {
        setWantedState(SuperstructureState.WantedState.INTAKE_CORAL);
      } else {
        setWantedState(SuperstructureState.WantedState.INTAKE_ALGAE_GROUND);
      }
    });
  }
  public Command intakeStopCommand() {
    return new InstantCommand(() -> {
      effector.setWantedState(EndEffector.WantedState.OFF);
      setWantedState(SuperstructureState.WantedState.DEFAULT);
    });
  }
  
  // Score-confirm (hold)
  public Command scoreStartCommand() {
    return new InstantCommand(() -> {
      if (mode == SuperstructureState.Mode.CORAL) {
        if (coralLevel == SuperstructurePose.ElevatorLevel.L1) {
          effector.setWantedState(EndEffector.WantedState.EJECT_CORAL_L1);
        } else {
          effector.setWantedState(EndEffector.WantedState.EJECT_CORAL_FORWARD);
        }
      } else {
        effector.setWantedState(EndEffector.WantedState.EJECT_ALGAE_PROCESSOR);
      }
    });
  }
  public Command scoreStopCommand() {
    return new InstantCommand(() -> {
      effector.setWantedState(EndEffector.WantedState.OFF);
      setWantedState(SuperstructureState.WantedState.STOW);
    });
  }

  public Command coralUnstuckCommand(){
    return new InstantCommand(); //TODO: Add EJECT_CORAL_BACKWARD state to END_EFFECTOR to set state.
  }

  private void toggleWristInternal() {
    // If stowed, go to hover for mode/level; otherwise go back to stow.
    boolean atStow = Math.abs(wrist.getPositionRads() - WristConstants.Preset.STOW.getRadians())
                 <= WristConstants.GOAL_TOLERANCE_RAD;
  
    if (mode == SuperstructureState.Mode.CORAL) {
      if (atStow) {
        // Hover presets per level (uses your Pose table)
        switch (coralLevel) {
          case L1 -> goToPose(SuperstructurePose.coralHover(SuperstructurePose.ElevatorLevel.L1, false));
          case L2 -> goToPose(SuperstructurePose.coralHover(SuperstructurePose.ElevatorLevel.L2, false));
          case L3 -> goToPose(SuperstructurePose.coralHover(SuperstructurePose.ElevatorLevel.L3, false));
          case L4 -> goToPose(SuperstructurePose.coralHover(SuperstructurePose.ElevatorLevel.L4, DriverStation.isAutonomous()));
          default -> goToPose(SuperstructurePose.STOW);
        }
      } else {
        setWantedState(SuperstructureState.WantedState.STOW);
      }
    } else { // ALGAE
      // TODO: Add Reef intake wrist toggle
      if (atStow) {
        goToPose(SuperstructurePose.ALGAE_GROUND_INTAKE);
      } else {
        setWantedState(SuperstructureState.WantedState.STOW);
      }
    }
  }
}
