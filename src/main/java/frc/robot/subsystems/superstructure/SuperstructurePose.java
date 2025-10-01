package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.wrist.WristConstants;

/**
 * Named superstructure poses (elevator + wrist), plus small pure helpers.
 *
 * <p>Device-agnostic: no mechanism logic or sensor reads. This class only carries data and policy.
 */
public final class SuperstructurePose {
  private SuperstructurePose() {}

  // ---------------------------------------------
  // Discrete enums - Defined states of subsystems
  // ---------------------------------------------

  /** Discrete elevator levels and their nominal heights (meters). */
  public enum ElevatorLevel {
    STOW(ElevatorConstants.Preset.STOW),
    L1(ElevatorConstants.Preset.L1),
    L2(ElevatorConstants.Preset.L2),
    L3(ElevatorConstants.Preset.L3),
    L4(ElevatorConstants.Preset.L4),
    ALGAE_LOW(ElevatorConstants.Preset.ALGAE_LOW),
    ALGAE_HIGH(ElevatorConstants.Preset.ALGAE_HIGH);

    /** Nominal elevator height in meters for this level. */
    public final double meters;

    /**
     * @param meters Nominal height in meters for this level.
     */
    ElevatorLevel(double meters) {
      this.meters = meters;
    }
  }

  /** Discrete wrist presets and their nominal angles. */
  public enum WristPreset {
    STOW(WristConstants.Preset.STOW),
    HOVER_L1(WristConstants.Preset.HOVER_L1),
    HOVER_L2(WristConstants.Preset.HOVER_L2),
    HOVER_L3(WristConstants.Preset.HOVER_L3),
    HOVER_L4(WristConstants.Preset.HOVER_L4),
    AUTO_HOVER_L4(WristConstants.Preset.AUTO_HOVER_L4),

    DUNK_L1(WristConstants.Preset.DUNK_L1),
    DUNK_L2(WristConstants.Preset.DUNK_L2),
    DUNK_L3(WristConstants.Preset.DUNK_L3),
    DUNK_L4(WristConstants.Preset.DUNK_L4),

    REEF_ALGAE_LOW(WristConstants.Preset.REEF_ALGAE_LOW),
    REEF_ALGAE_HIGH(WristConstants.Preset.REEF_ALGAE_HIGH),
    GROUND_ALGAE(WristConstants.Preset.GROUND_ALGAE),
    PROCESSOR(WristConstants.Preset.PROCESSOR);

    /** Nominal wrist angle for this preset. */
    public final Rotation2d angle;

    /**
     * @param angle Nominal {@link Rotation2d} for this preset.
     */
    WristPreset(Rotation2d angle) {
      this.angle = angle;
    }
  }

  // -------------------------
  // Single Pose type
  // -------------------------

  /**
   * A superstructure pose = (elevator target, wrist target).
   *
   * <p>Two construction modes:
   *
   * <ul>
   *   <li><b>DISCRETE</b>: built with both enums (ElevatorLevel + WristPreset) → safe for policy &
   *       setPose.
   *   <li><b>RAW</b>: any constructor that does not provide both enums → enums are {@code null};
   *       not safe for setPose.
   * </ul>
   *
   * <p>Effective numeric targets are always available.
   */
  public static final class Pose {
    private final ElevatorLevel level; // null if RAW
    private final WristPreset wristPreset; // null if RAW

    private final double elevatorMeters; // always present
    private final Rotation2d wristAngle; // always present

    // ===== Constructors YOU requested (ONLY these 6) =====

    /**
     * DISCRETE: constructs from enums (level + preset).
     *
     * @param level Elevator level (must be non-null).
     * @param preset Wrist preset (must be non-null).
     * @throws IllegalArgumentException if any argument is null.
     */
    public Pose(ElevatorLevel level, WristPreset preset) {
      if (level == null || preset == null)
        throw new IllegalArgumentException("Both enums required");
      this.level = level;
      this.wristPreset = preset;
      this.elevatorMeters = level.meters;
      this.wristAngle = preset.angle;
    }

    /**
     * RAW: constructs from numeric elevator + wrist preset.
     *
     * @param elevatorMeters Elevator target in meters.
     * @param preset Wrist preset (used only to get a numeric angle; enum is not kept).
     * @throws IllegalArgumentException if preset is null.
     */
    public Pose(double elevatorMeters, WristPreset preset) {
      if (preset == null) throw new IllegalArgumentException("WristPreset cannot be null");
      this.level = null; // mark RAW
      this.wristPreset = null;
      this.elevatorMeters = elevatorMeters;
      this.wristAngle = preset.angle;
    }

    /**
     * RAW: constructs from elevator level + numeric wrist radians.
     *
     * @param level Elevator level (used only to get a numeric height; enum is not kept).
     * @param wristRadians Wrist target in radians.
     * @throws IllegalArgumentException if level is null.
     */
    public Pose(ElevatorLevel level, double wristRadians) {
      if (level == null) throw new IllegalArgumentException("ElevatorLevel cannot be null");
      this.level = null; // mark RAW
      this.wristPreset = null;
      this.elevatorMeters = level.meters;
      this.wristAngle = Rotation2d.fromRadians(wristRadians);
    }

    /**
     * RAW: constructs from numeric elevator + numeric wrist radians.
     *
     * @param elevatorMeters Elevator target in meters.
     * @param wristRadians Wrist target in radians.
     */
    public Pose(double elevatorMeters, double wristRadians) {
      this.level = null; // mark RAW
      this.wristPreset = null;
      this.elevatorMeters = elevatorMeters;
      this.wristAngle = Rotation2d.fromRadians(wristRadians);
    }

    /**
     * RAW: constructs from numeric elevator + wrist angle.
     *
     * @param elevatorMeters Elevator target in meters.
     * @param wristAngle Wrist target angle.
     * @throws IllegalArgumentException if wristAngle is null.
     */
    public Pose(double elevatorMeters, Rotation2d wristAngle) {
      if (wristAngle == null) throw new IllegalArgumentException("wristAngle cannot be null");
      this.level = null; // mark RAW
      this.wristPreset = null;
      this.elevatorMeters = elevatorMeters;
      this.wristAngle = wristAngle;
    }

    /**
     * RAW: constructs from elevator level + wrist angle.
     *
     * @param level Elevator level (used only to get a numeric height; enum is not kept).
     * @param wristAngle Wrist target angle.
     * @throws IllegalArgumentException if any argument is null.
     */
    public Pose(ElevatorLevel level, Rotation2d wristAngle) {
      if (level == null || wristAngle == null)
        throw new IllegalArgumentException("args cannot be null");
      this.level = null; // mark RAW
      this.wristPreset = null;
      this.elevatorMeters = level.meters;
      this.wristAngle = wristAngle;
    }

    // ---- Effective numeric targets (what you command) ----

    /**
     * @return Elevator target height (meters).
     */
    public double elevatorMeters() {
      return elevatorMeters;
    }

    /**
     * @return Wrist target angle.
     */
    public Rotation2d wristAngle() {
      return wristAngle;
    }

    // ---- Policy flags/accessors ----

    /**
     * @return True if constructed with both enums (safe for {@code setPose}).
     */
    public boolean isDiscrete() {
      return level != null && wristPreset != null;
    }

    /**
     * @return The discrete elevator level if present; otherwise {@code null}.
     */
    public ElevatorLevel levelOrNull() {
      return level;
    }

    /**
     * @return The discrete wrist preset if present; otherwise {@code null}.
     */
    public WristPreset wristPresetOrNull() {
      return wristPreset;
    }

    // ---- Minimal helpers ----

    /**
     * Returns the elevator level; throws if RAW.
     *
     * @return Elevator level.
     * @throws IllegalStateException if pose is RAW.
     */
    public ElevatorLevel requireLevel() {
      if (level == null) throw new IllegalStateException("Pose is RAW (no ElevatorLevel).");
      return level;
    }

    /**
     * Returns the wrist preset; throws if RAW.
     *
     * @return Wrist preset.
     * @throws IllegalStateException if pose is RAW.
     */
    public WristPreset requireWristPreset() {
      if (wristPreset == null) throw new IllegalStateException("Pose is RAW (no WristPreset).");
      return wristPreset;
    }

    /**
     * Tolerant numeric comparison vs another pose.
     *
     * @param other Other pose to compare.
     * @param elevTolMeters Elevator tolerance (meters).
     * @param wristTolRad Wrist tolerance (radians).
     * @return True if both numeric axes are within tolerances.
     */
    public boolean approxEquals(Pose other, double elevTolMeters, double wristTolRad) {
      if (other == null) return false;
      return Math.abs(this.elevatorMeters - other.elevatorMeters) <= elevTolMeters
          && Math.abs(this.wristAngle.getRadians() - other.wristAngle.getRadians()) <= wristTolRad;
    }

    /**
     * Same elevator LEVEL (DISCRETE only).
     *
     * @param other Other pose to compare.
     * @return True if both poses are DISCRETE and share the same level; false otherwise.
     */
    public boolean sameLevelDiscrete(Pose other) {
      return other != null
          && this.level != null
          && other.level != null
          && this.level == other.level;
    }

    /**
     * Travel-safe copy: wrist stowed, elevator unchanged (RAW or DISCRETE both supported).
     *
     * @return A new pose with the same elevator target and STOW wrist angle.
     */
    public Pose stowedWrist() {
      return new Pose(this.elevatorMeters, WristConstants.Preset.STOW);
    }
  }

  // -------------------------
  // Named discrete poses
  // -------------------------

  /** STOW pose (elevator STOW, wrist STOW). */
  public static final Pose STOW = new Pose(ElevatorLevel.STOW, WristPreset.STOW);

  /**
   * Coral hover pose for a given level.
   *
   * @param level Elevator level (L1–L4).
   * @param isAuto Whether auto-hover should be used for L4 in autonomous.
   * @return A discrete hover pose for the specified level.
   */
  public static Pose coralHover(ElevatorLevel level, boolean isAuto) {
    return switch (level) {
      case L1 -> new Pose(ElevatorLevel.L1, WristPreset.HOVER_L1);
      case L2 -> new Pose(ElevatorLevel.L2, WristPreset.HOVER_L2);
      case L3 -> new Pose(ElevatorLevel.L3, WristPreset.HOVER_L3);
      case L4 -> new Pose(
          ElevatorLevel.L4, isAuto ? WristPreset.AUTO_HOVER_L4 : WristPreset.HOVER_L4);
      default -> STOW;
    };
  }

  /**
   * Coral dunk pose for a given level.
   *
   * @param level Elevator level (L1–L4).
   * @return A discrete dunk pose for the specified level.
   */
  public static Pose coralDunk(ElevatorLevel level) {
    return switch (level) {
      case L1 -> new Pose(ElevatorLevel.L1, WristPreset.DUNK_L1);
      case L2 -> new Pose(ElevatorLevel.L2, WristPreset.DUNK_L2);
      case L3 -> new Pose(ElevatorLevel.L3, WristPreset.DUNK_L3);
      case L4 -> new Pose(ElevatorLevel.L4, WristPreset.DUNK_L4);
      default -> STOW;
    };
  }

  // Algae intake / travel / processor

  /** Ground-intake pose for algae (elevator STOW, wrist GROUND_ALGAE). */
  public static final Pose ALGAE_GROUND_INTAKE =
      new Pose(ElevatorLevel.STOW, WristPreset.GROUND_ALGAE);

  /** Reef low-intake pose for algae (elevator ALGAE_LOW, wrist REEF_ALGAE_LOW). */
  public static final Pose ALGAE_REEF_LOW_INTAKE =
      new Pose(ElevatorLevel.ALGAE_LOW, WristPreset.REEF_ALGAE_LOW);

  /** Reef high-intake pose for algae (elevator ALGAE_HIGH, wrist REEF_ALGAE_HIGH). */
  public static final Pose ALGAE_REEF_HIGH_INTAKE =
      new Pose(ElevatorLevel.ALGAE_HIGH, WristPreset.REEF_ALGAE_HIGH);

  /** Algae travel/hold pose (elevator STOW, wrist STOW). */
  public static final Pose ALGAE_TRAVEL_HOLD = new Pose(ElevatorLevel.STOW, WristPreset.STOW);

  /** Processor pose for algae (elevator STOW, wrist PROCESSOR). */
  public static final Pose ALGAE_AT_PROCESSOR = new Pose(ElevatorLevel.STOW, WristPreset.PROCESSOR);

  /**
   * Centralized wrist-toggle policy.
   *
   * @param mode Current superstructure mode (CORAL/ALGAE).
   * @param coralLevel Current coral level selection (used for CORAL).
   * @param isAuto True if in autonomous period (enables auto-hover L4 policy).
   * @param hasAlgae True if a piece of algae is detected (used for ALGAE toggle-down).
   * @param goingDown True to produce a "down" pose (dunk/ground/proc), false for a "up/hover" pose.
   * @return A discrete pose to feed into {@code setPose(...)}.
   */
  public static Pose toggleWrist(
      SuperstructureState.Mode mode,
      ElevatorLevel coralLevel,
      boolean isAuto,
      boolean hasAlgae,
      boolean goingDown) {

    if (mode == SuperstructureState.Mode.CORAL) {
      return goingDown
          ? coralDunk(coralLevel)
          : coralHover(coralLevel, isAuto && coralLevel == ElevatorLevel.L4);
    } else { // ALGAE
      return goingDown ? (hasAlgae ? ALGAE_AT_PROCESSOR : ALGAE_GROUND_INTAKE) : ALGAE_TRAVEL_HOLD;
    }
  }

  /**
   * Travel-safe helper from a level (keeps elevator discrete, wrist STOW).
   *
   * @param level Discrete elevator level.
   * @return A discrete pose with the same level and wrist STOW.
   */
  public static Pose wristStowForTravel(ElevatorLevel level) {
    return new Pose(level, WristPreset.STOW);
  }

  /**
   * Travel-safe helper from any pose (keeps the numeric elevator target, wrist STOW).
   *
   * @param pose Any pose (RAW or DISCRETE).
   * @return A new pose with the same numeric elevator target and wrist STOW.
   */
  public static Pose wristStowForTravel(Pose pose) {
    return pose.stowedWrist();
  }
}
