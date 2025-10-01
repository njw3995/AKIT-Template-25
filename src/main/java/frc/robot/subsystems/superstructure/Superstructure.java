package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.superstructure.SuperstructurePose.ElevatorLevel;
import frc.robot.subsystems.superstructure.SuperstructurePose.Pose;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.util.FieldConstants;
import frc.robot.util.OperatorDashboard;
import org.littletonrobotics.junction.Logger;

/**
 * 2910-style high-level coordinator for the elevator + wrist + end effector.
 *
 * <p>Lifecycle: {@code periodic() -> handleStateTransitions() -> applyStates()}.
 *
 * <ul>
 *   <li>No device constants here; all named setpoints live in {@link SuperstructurePose}.
 *   <li>Multi-axis sequencing is handled in {@link #setPose(Pose)}.
 *   <li>Reef auto-align (LEFT/RIGHT) is initiated/cancelled by commands on this subsystem.
 * </ul>
 */
public class Superstructure extends SubsystemBase {
  // -----------------------
  // Subsystems & UI inputs
  // -----------------------
  private final Drive drive;
  private final Elevator elevator;
  private final Wrist wrist;
  private final EndEffector effector;
  private final OperatorDashboard operatorDashboard;

  // -----------------------
  // Policy / automation
  // -----------------------
  private SuperstructureState.AutomationLevel automationLevel =
      SuperstructureState.AutomationLevel.AUTO_RELEASE;

  /** 2910-style ejection latch. */
  private boolean coralEjectFlag = false;

  // -----------------------
  // Alignment (reef)
  // -----------------------
  private boolean alignActive = false;
  private FieldConstants.ReefSide alignSide = FieldConstants.ReefSide.LEFT;

  /** Position & heading tolerances for considering the robot "aligned" to the reef target. */
  private static final double ALIGN_TRANS_TOL_M = 0.06; // ~6 cm

  private static final double ALIGN_ROT_TOL_RAD = Math.toRadians(4.0); // ~4°

  // -----------------------
  // State machine
  // -----------------------
  private SuperstructureState.WantedState wantedState = SuperstructureState.WantedState.DEFAULT;
  private SuperstructureState.CurrentState currentState = SuperstructureState.CurrentState.STOPPED;
  private SuperstructureState.CurrentState previousState = SuperstructureState.CurrentState.STOPPED;

  /** Last discrete pose we commanded via {@link #setPose(Pose)}. */
  private Pose lastCommandedPose = SuperstructurePose.STOW;

  private SuperstructureState.Mode mode = SuperstructureState.Mode.CORAL;
  private ElevatorLevel coralLevel = ElevatorLevel.L4; // operator-selected deployed level

  /**
   * Creates the superstructure coordinator.
   *
   * @param drive The drivetrain used for reef alignment targeting (goal publishing).
   * @param elevator The elevator subsystem.
   * @param wrist The wrist subsystem.
   * @param effector The end-effector subsystem (coral/algae).
   * @param operatorDashboard Dashboard for automation level selection.
   */
  public Superstructure(
      Drive drive,
      Elevator elevator,
      Wrist wrist,
      EndEffector effector,
      OperatorDashboard operatorDashboard) {
    this.drive = drive;
    this.elevator = elevator;
    this.wrist = wrist;
    this.effector = effector;
    this.operatorDashboard = operatorDashboard;
  }

  // -----------------------
  // External API
  // -----------------------

  /**
   * Sets the operator intent (wanted high-level state).
   *
   * @param state New wanted state.
   */
  public void setWantedState(SuperstructureState.WantedState state) {
    wantedState = state;
  }

  /**
   * Button-friendly wrapper for {@link #setWantedState(SuperstructureState.WantedState)}.
   *
   * @param state New wanted state.
   * @return Command that sets the wanted state when scheduled.
   */
  public Command setStateCommand(SuperstructureState.WantedState state) {
    return new InstantCommand(() -> setWantedState(state), this);
  }

  /** Toggles between CORAL and ALGAE modes. */
  public void toggleMode() {
    mode =
        (mode == SuperstructureState.Mode.CORAL)
            ? SuperstructureState.Mode.ALGAE
            : SuperstructureState.Mode.CORAL;
  }

  /**
   * Sets the operator-selected coral level (L1–L4).
   *
   * @param level New coral level used by {@link #requestDeploy()} and scoring states.
   */
  public void setCoralLevel(ElevatorLevel level) {
    coralLevel = level;
    Logger.recordOutput("Superstructure/CoralLevel", coralLevel);
  }

  /**
   * Requests a deploy action based on current {@link #mode} and {@link #coralLevel}.
   *
   * <ul>
   *   <li>CORAL → SCORE_Lx
   *   <li>ALGAE → INTAKE_ALGAE_REEF
   * </ul>
   */
  public void requestDeploy() {
    if (mode == SuperstructureState.Mode.CORAL) {
      switch (coralLevel) {
        case L1 -> setWantedState(SuperstructureState.WantedState.SCORE_L1);
        case L2 -> setWantedState(SuperstructureState.WantedState.SCORE_L2);
        case L3 -> setWantedState(SuperstructureState.WantedState.SCORE_L3);
        case L4 -> setWantedState(SuperstructureState.WantedState.SCORE_L4);
        default -> setWantedState(SuperstructureState.WantedState.STOW);
      }
    } else { // ALGAE
      setWantedState(SuperstructureState.WantedState.INTAKE_ALGAE_REEF);
    }
  }

  // -----------------------
  // Commands (for OI)
  // -----------------------

  /**
   * Command: toggle CORAL/ALGAE mode.
   *
   * @return Command that toggles mode when scheduled.
   */
  public Command toggleModeCommand() {
    return new InstantCommand(this::toggleMode, this);
  }

  /**
   * Command: wrist toggle using {@link SuperstructurePose#toggleWrist}.
   *
   * <ul>
   *   <li>CORAL: if wrist is stowed, go to HOVER (L4 auto-hover in auto); else STOW.
   *   <li>ALGAE: if wrist is stowed, go to GROUND_INTAKE or PROCESSOR (depending on piece); else
   *       STOW.
   * </ul>
   *
   * @return Command that toggles the wrist pose when scheduled.
   */
  public Command wristToggleCommand() {
    return new InstantCommand(this::wristToggle, this);
  }

  /**
   * Command: set the coral deploy level.
   *
   * @param lvl Level (L1–L4).
   * @return Command that sets {@link #coralLevel} when scheduled.
   */
  public Command setDeployedCoralLevelCommand(ElevatorLevel lvl) {
    return new InstantCommand(() -> setCoralLevel(lvl), this);
  }

  /**
   * Command: request deploy based on mode/level.
   *
   * @return Command that calls {@link #requestDeploy()} when scheduled.
   */
  public Command deployCommand() {
    return new InstantCommand(this::requestDeploy, this);
  }

  /**
   * Command: stow (via wanted state).
   *
   * @return Command that sets wanted state to STOW when scheduled.
   */
  public Command stowCommand() {
    return setStateCommand(SuperstructureState.WantedState.STOW);
  }

  /**
   * Command: start intake (mode-aware).
   *
   * @return Command that switches to the appropriate intake state when scheduled.
   */
  public Command intakeStartCommand() {
    return new InstantCommand(
        () -> {
          if (mode == SuperstructureState.Mode.CORAL) {
            setWantedState(SuperstructureState.WantedState.INTAKE_CORAL);
          } else {
            setWantedState(SuperstructureState.WantedState.INTAKE_ALGAE_GROUND);
          }
        },
        this);
  }

  /**
   * Command: stop intake and return to DEFAULT.
   *
   * @return Command that stops the effector and returns to DEFAULT when scheduled.
   */
  public Command intakeStopCommand() {
    return new InstantCommand(
        () -> {
          effector.setWantedState(EndEffector.WantedState.OFF);
          setWantedState(SuperstructureState.WantedState.DEFAULT);
        },
        this);
  }

  /**
   * Command: hold-to-align LEFT to reef (alignment managed in {@link #periodic()}).
   *
   * @return Hold command that starts alignment on init and stops on end.
   */
  public Command alignLeftHoldCommand() {
    return Commands.startEnd(() -> startAlign(FieldConstants.ReefSide.LEFT), this::stopAlign, this);
  }

  /**
   * Command: hold-to-align RIGHT to reef (alignment managed in {@link #periodic()}).
   *
   * @return Hold command that starts alignment on init and stops on end.
   */
  public Command alignRightHoldCommand() {
    return Commands.startEnd(
        () -> startAlign(FieldConstants.ReefSide.RIGHT), this::stopAlign, this);
  }

  /**
   * Command: manual score trigger (press-and-hold to eject based on current mode/level). Leaves the
   * automation (auto-release) untouched.
   *
   * @return Command that sets an eject state while held.
   */
  public Command scoreStartCommand() {
    return new InstantCommand(
        () -> {
          if (mode == SuperstructureState.Mode.CORAL) {
            if (coralLevel == ElevatorLevel.L1) {
              effector.setWantedState(EndEffector.WantedState.EJECT_CORAL_L1);
            } else {
              effector.setWantedState(EndEffector.WantedState.EJECT_CORAL_FORWARD);
            }
          } else {
            effector.setWantedState(EndEffector.WantedState.EJECT_ALGAE_PROCESSOR);
          }
        },
        this);
  }

  /**
   * Command: release manual score trigger; return to STOW.
   *
   * @return Command that turns the effector OFF and sets wanted state STOW.
   */
  public Command scoreStopCommand() {
    return new InstantCommand(
        () -> {
          effector.setWantedState(EndEffector.WantedState.OFF);
          setWantedState(SuperstructureState.WantedState.STOW);
        },
        this);
  }

  // -----------------------
  // Scheduler entry
  // -----------------------
  @Override
  public void periodic() {
    // Dashboard automation setting
    automationLevel = operatorDashboard.getAutomationLevel();

    currentState = handleStateTransitions(wantedState);
    applyStates(currentState);

    // Publish target to drive while aligning (drive decides how to use the goal)
    if (alignActive) {
      Pose2d target = reefAlignTarget(alignSide);
      // drive.setAutoAlignGoal(target);
      // drive.enableAutoAlign();
      Logger.recordOutput("Superstructure/ReefAlign/Target", target);
    }

    Logger.recordOutput("Superstructure/AutomationLevel", automationLevel.toString());
    Logger.recordOutput("Superstructure/WantedState", wantedState.toString());
    Logger.recordOutput("Superstructure/CurrentState", currentState.toString());
    Logger.recordOutput("Superstructure/PreviousState", previousState.toString());
  }

  // -----------------------
  // Transitions (Wanted -> Current)
  // -----------------------

  /**
   * Maps operator intent to an internal state; no mechanism calls here.
   *
   * @param state The wanted state to map.
   * @return The internal current-state that should be applied this cycle.
   */
  private SuperstructureState.CurrentState handleStateTransitions(
      SuperstructureState.WantedState state) {
    switch (state) {
      case STOP:
        return SuperstructureState.CurrentState.STOPPED;
      case HOME:
        return SuperstructureState.CurrentState.HOMING;
      case STOW:
        return SuperstructureState.CurrentState.STOWING;

      case DEFAULT:
        if (effector.hasCoral()) return SuperstructureState.CurrentState.HOLDING_CORAL;
        if (effector.hasAlgae()) return SuperstructureState.CurrentState.HOLDING_ALGAE;
        return SuperstructureState.CurrentState.STOWING;

        // Coral
      case INTAKE_CORAL:
        return SuperstructureState.CurrentState.INTAKING_CORAL;
      case HOLD_CORAL:
        return SuperstructureState.CurrentState.HOLDING_CORAL;
      case SCORE_L1:
        return SuperstructureState.CurrentState.SCORING_L1;
      case SCORE_L2:
        return SuperstructureState.CurrentState.SCORING_L2;
      case SCORE_L3:
        return SuperstructureState.CurrentState.SCORING_L3;
      case SCORE_L4:
        return SuperstructureState.CurrentState.SCORING_L4;

        // Algae
      case INTAKE_ALGAE_GROUND:
        return SuperstructureState.CurrentState.INTAKING_ALGAE_GROUND;
      case INTAKE_ALGAE_REEF:
        return SuperstructureState.CurrentState.INTAKING_ALGAE_REEF;
      case HOLD_ALGAE:
        return SuperstructureState.CurrentState.HOLDING_ALGAE;
      case SCORE_ALGAE_PROCESSOR:
        return SuperstructureState.CurrentState.SCORING_ALGAE_PROCESSOR;

      default:
        return SuperstructureState.CurrentState.STOPPED;
    }
  }

  // -----------------------
  // Apply (Current -> actions)
  // -----------------------

  /**
   * Applies mechanism actions for the given current state.
   *
   * @param state The internal current state to execute.
   */
  private void applyStates(SuperstructureState.CurrentState state) {
    previousState = state;
    switch (state) {
      case STOPPED -> stop();
      case HOMING -> home();
      case STOWING -> stow();

        // Coral
      case INTAKING_CORAL -> intakeCoral();
      case HOLDING_CORAL -> holdCoral();
      case SCORING_L1 -> scoreCoral(ElevatorLevel.L1);
      case SCORING_L2 -> scoreCoral(ElevatorLevel.L2);
      case SCORING_L3 -> scoreCoral(ElevatorLevel.L3);
      case SCORING_L4 -> scoreCoral(ElevatorLevel.L4);

        // Algae
      case INTAKING_ALGAE_GROUND -> intakeAlgaeGround();
      case INTAKING_ALGAE_REEF -> intakeAlgaeReef();
      case HOLDING_ALGAE -> holdAlgae();
      case SCORING_ALGAE_PROCESSOR -> scoreAlgaeProcessor();
    }
  }

  // -----------------------
  // Helpers & sequencing
  // -----------------------

  /**
   * Returns whether the robot is within tolerance of the given pose, using live sensors.
   *
   * @param p Pose to compare against.
   * @return True if both elevator and wrist are within goal tolerances.
   */
  private boolean atPose(Pose p) {
    return Math.abs(elevator.getPositionMeters() - p.elevatorMeters())
            <= ElevatorConstants.GOAL_TOLERANCE_M
        && Math.abs(wrist.getPositionRads() - p.wristAngle().getRadians())
            <= WristConstants.GOAL_TOLERANCE_RAD;
  }

  /**
   * Multi-axis waypoint sequencer.
   *
   * <p>Given a discrete target pose:
   *
   * <ol>
   *   <li>If the elevator level differs, go: current → current_stowed → target_stowed → target.
   *   <li>If level is the same, go: current → target_stowed → target.
   *   <li>Each tick selects exactly one waypoint and commands it.
   * </ol>
   *
   * @param target Discrete target pose (must be constructed with enums).
   */
  private void setPose(Pose target) {
    // Safety: only discrete → full coordinator sequencing
    if (!lastCommandedPose.isDiscrete() || !target.isDiscrete()) {
      return; // avoid undefined behavior with RAW poses
    }

    final Pose current = lastCommandedPose;
    final Pose currentStowed = current.stowedWrist();
    final Pose targetStowed = target.stowedWrist();

    final boolean levelChanged = !current.sameLevelDiscrete(target);
    final boolean atCurrentStowed = atPose(currentStowed);
    final boolean atTargetStowed = atPose(targetStowed);

    // Choose exactly one waypoint each tick
    final Pose waypoint;
    if (!levelChanged) {
      // Same level: stay stowed until targetStowed is actually reached, then finish to target
      waypoint = atTargetStowed ? target : targetStowed;
    } else {
      // Different level: stow at the *current* level first, else drive toward *target* stowed
      waypoint = !atCurrentStowed ? currentStowed : targetStowed;
    }

    // Command the chosen waypoint or HOLD if at waypoint
    if (atPose(waypoint)) {
      elevator.setWantedState(Elevator.WantedState.HOLD);
      wrist.setWantedState(Wrist.WantedState.HOLD);
    } else {
      elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, waypoint.elevatorMeters());
      wrist.setWantedState(Wrist.WantedState.MOVE_TO_POSITION, waypoint.wristAngle());
    }
    // Reflect what we're actually driving to
    lastCommandedPose = waypoint;
  }

  /** IDLE all subsystems. */
  private void stop() {
    elevator.setWantedState(Elevator.WantedState.IDLE);
    wrist.setWantedState(Wrist.WantedState.IDLE);
    effector.setWantedState(EndEffector.WantedState.OFF);
  }

  /** Simple homing gate: consider homed when both axes are near STOW setpoints. */
  private void home() {
    boolean elevatorAtStow =
        Math.abs(elevator.getPositionMeters() - ElevatorConstants.Preset.STOW)
            <= ElevatorConstants.GOAL_TOLERANCE_M;
    boolean wristAtStow =
        Math.abs(wrist.getPositionRads() - WristConstants.Preset.STOW.getRadians())
            <= WristConstants.GOAL_TOLERANCE_RAD;

    if (elevatorAtStow && wristAtStow) {
      setWantedState(SuperstructureState.WantedState.DEFAULT);
    }
  }

  /** Drives to the STOW pose using the waypoint sequencer. */
  private void stow() {
    setPose(SuperstructurePose.STOW);
  }

  // --- Coral ---
  /** Sets the end effector to collect coral (mechanisms stay wherever they are). */
  private void intakeCoral() {
    effector.setWantedState(EndEffector.WantedState.COLLECT_CORAL);
  }

  /** Holds the coral with the end effector. */
  private void holdCoral() {
    effector.setWantedState(EndEffector.WantedState.HOLD_CORAL);
  }

  /**
   * Score coral workflow for a specific elevator level (L1–L4).
   *
   * <p>Uses hover in teleop (or auto-hover for L4 in auto), with 2910-style eject latch.
   *
   * @param level Level being scored.
   */
  private void scoreCoral(ElevatorLevel level) {
    final Pose pose =
        (level == ElevatorLevel.L4 && DriverStation.isAutonomous())
            ? SuperstructurePose.coralHover(ElevatorLevel.L4, true)
            : SuperstructurePose.coralHover(level, false);
    setPose(pose);

    // When both drive alignment & mechanism are ready, arm the latch (2910 style)
    if ((DriverStation.isAutonomous() ? isReadyToEjectAuto() : isReadyToEjectTeleop())) {
      coralEjectFlag = true;
    }

    // If latched and automation is enabled, eject once
    if (coralEjectFlag) {
      if (automationLevel == SuperstructureState.AutomationLevel.AUTO_RELEASE) {
        if (level == ElevatorLevel.L1) {
          effector.setWantedState(EndEffector.WantedState.EJECT_CORAL_L1);
        } else {
          effector.setWantedState(EndEffector.WantedState.EJECT_CORAL_FORWARD);
        }
      } else {
        // Manual mode: driver presses your score button to eject; keep holding here.
        effector.setWantedState(EndEffector.WantedState.HOLD_CORAL);
      }

      // Optional: autostow after scoring
      // if (!effector.hasCoral()) {
      //   coralEjectFlag = false;
      //   setWantedState(SuperstructureState.WantedState.STOW);
      // }
    } else {
      effector.setWantedState(EndEffector.WantedState.HOLD_CORAL);
    }
  }

  // --- Algae ---
  /** Drives to ground-intake pose, starts algae collection, and switches to HOLD when acquired. */
  private void intakeAlgaeGround() {
    setPose(SuperstructurePose.ALGAE_GROUND_INTAKE);
    effector.setWantedState(EndEffector.WantedState.COLLECT_ALGAE);
    if (effector.hasAlgae()) setWantedState(SuperstructureState.WantedState.HOLD_ALGAE);
  }

  /**
   * Drives to reef low-intake pose, starts algae collection, and switches to HOLD when acquired.
   */
  private void intakeAlgaeReef() {
    setPose(SuperstructurePose.ALGAE_REEF_LOW_INTAKE);
    effector.setWantedState(EndEffector.WantedState.COLLECT_ALGAE);
    if (effector.hasAlgae()) setWantedState(SuperstructureState.WantedState.HOLD_ALGAE);
  }

  /** Drives to travel/hold pose and holds algae in the end effector. */
  private void holdAlgae() {
    setPose(SuperstructurePose.ALGAE_TRAVEL_HOLD);
    effector.setWantedState(EndEffector.WantedState.HOLD_ALGAE);
  }

  /** Drives to processor pose; ejects when ready else keeps holding. */
  private void scoreAlgaeProcessor() {
    setPose(SuperstructurePose.ALGAE_AT_PROCESSOR);
    if (isReadyToScore()) {
      effector.setWantedState(EndEffector.WantedState.EJECT_ALGAE_PROCESSOR);
    } else {
      effector.setWantedState(EndEffector.WantedState.HOLD_ALGAE);
    }
  }

  /**
   * Returns whether both elevator and wrist have reached their internal setpoints.
   *
   * @return True if both reached their setpoints.
   */
  private boolean isReadyToScore() {
    return elevator.reachedSetpoint() && wrist.reachedSetpoint();
  }

  // -----------------------
  // Wrist toggle (policy)
  // -----------------------

  /**
   * Implements {@link #wristToggleCommand()} using centralized policy in {@link
   * SuperstructurePose}.
   */
  private void wristToggle() {
    final boolean isAuto = DriverStation.isAutonomous();
    final boolean hasAlgae = effector.hasAlgae();

    // Are we currently "at stow" (within tolerance) on the wrist? (elevator doesn't matter here)
    final boolean wristAtStow =
        Math.abs(wrist.getPositionRads() - WristConstants.Preset.STOW.getRadians())
            <= WristConstants.GOAL_TOLERANCE_RAD;

    // goingDown => choose "dunk/ground/proc" vs "hover/hold"
    final boolean goingDown =
        !wristAtStow; // if not stowed -> we toggle down to stow/dunk; if stowed -> up to
    // hover/intake

    final Pose next = SuperstructurePose.toggleWrist(mode, coralLevel, isAuto, hasAlgae, goingDown);
    setPose(next);
  }

  // -----------------------
  // Reef alignment helpers
  // -----------------------

  /**
   * Starts reef alignment toward a side (LEFT/RIGHT). The target is computed each loop from the
   * robot's live pose to the nearest reef face and branch for that side.
   *
   * @param side LEFT or RIGHT side of the nearest reef face.
   */
  public void startAlign(FieldConstants.ReefSide side) {
    alignSide = side;
    alignActive = true;
  }

  /** Stops reef alignment. */
  public void stopAlign() {
    alignActive = false;
    // drive.disableAutoAlign(); // optional
  }

  /**
   * Computes the reef alignment goal (2D) for the given side and the robot's current field pose.
   *
   * @param side LEFT or RIGHT side of the nearest reef face.
   * @return The target field pose to align to.
   */
  private Pose2d reefAlignTarget(FieldConstants.ReefSide side) {
    Pose2d robot = frc.robot.RobotState.getInstance().getRobotPoseField();
    return FieldConstants.getNearestReefBranch(robot, side);
  }

  /**
   * Returns whether the robot is aligned to the reef target under the configured tolerances.
   *
   * @param side LEFT or RIGHT side (target computed from live pose).
   * @return True if both translational and rotational error are within tolerances.
   */
  private boolean isAlignedToReef(FieldConstants.ReefSide side) {
    Pose2d robot = frc.robot.RobotState.getInstance().getRobotPoseField();
    Pose2d goal = reefAlignTarget(side);

    Transform2d err = goal.minus(robot);
    double posErr = Math.hypot(err.getX(), err.getY());
    double rotErr = Math.abs(err.getRotation().getRadians());

    Logger.recordOutput("Superstructure/ReefAlign/PosErrM", posErr);
    Logger.recordOutput("Superstructure/ReefAlign/RotErrRad", rotErr);

    return posErr <= ALIGN_TRANS_TOL_M && rotErr <= ALIGN_ROT_TOL_RAD;
  }

  /**
   * Teleop "ready-to-eject" gate matching 2910-style logic: robot aligned & mechanisms at pose.
   *
   * @return True if aligned and mechanisms are at setpoint.
   */
  private boolean isReadyToEjectTeleop() {
    return isAlignedToReef(alignSide) && isReadyToScore();
  }

  /**
   * Auto-period flavor for completeness (add path end checks if desired).
   *
   * @return True if aligned and mechanisms are at setpoint (and optionally path done).
   */
  private boolean isReadyToEjectAuto() {
    return isAlignedToReef(alignSide) && isReadyToScore(); // && drive.isAtEndOfPath();
  }
}
