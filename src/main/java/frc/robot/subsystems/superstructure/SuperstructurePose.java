package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.wrist.WristConstants;

/**
 * Named superstructure poses = (elevator height, wrist angle).
 * No mechanism logic here, no end-effector modes. Pure data.
 */
public final class SuperstructurePose {
  private SuperstructurePose() {}

  /** Discrete elevator setpoints you plan to use (meters pulled from ElevatorConstants). */
  public enum ElevatorLevel {
    STOW       (ElevatorConstants.Preset.STOW),
    L1         (ElevatorConstants.Preset.L1),
    L2         (ElevatorConstants.Preset.L2),
    L3         (ElevatorConstants.Preset.L3),
    L4         (ElevatorConstants.Preset.L4),
    ALGAE_LOW  (ElevatorConstants.Preset.ALGAE_LOW),
    ALGAE_HIGH (ElevatorConstants.Preset.ALGAE_HIGH);

    public final double meters;
    ElevatorLevel(double meters) { this.meters = meters; }
  }

  /** Discrete wrist angles you plan to use (Rotation2d pulled from WristConstants). */
  public enum WristPreset {
    STOW            (WristConstants.Preset.STOW),
    HOVER_L1        (WristConstants.Preset.HOVER_L1),
    HOVER_L2        (WristConstants.Preset.HOVER_L2),
    HOVER_L3        (WristConstants.Preset.HOVER_L3),
    HOVER_L4        (WristConstants.Preset.HOVER_L4),
    AUTO_HOVER_L4   (WristConstants.Preset.AUTO_HOVER_L4),

    DUNK_L1         (WristConstants.Preset.DUNK_L1),
    DUNK_L2         (WristConstants.Preset.DUNK_L2),
    DUNK_L3         (WristConstants.Preset.DUNK_L3),
    DUNK_L4         (WristConstants.Preset.DUNK_L4),

    REEF_ALGAE_LOW  (WristConstants.Preset.REEF_ALGAE_LOW),
    REEF_ALGAE_HIGH (WristConstants.Preset.REEF_ALGAE_HIGH),
    GROUND_ALGAE    (WristConstants.Preset.GROUND_ALGAE),
    PROCESSOR (WristConstants.Preset.PROCESSOR);

    public final Rotation2d angle;
    WristPreset(Rotation2d angle) { this.angle = angle; }
  }

  /** Immutable pairing of elevator+wrist. Superstructure uses this to command subsystems. */
  public static record Pose(ElevatorLevel elevator, WristPreset wrist) {}

  // -------------------------
  // Common, reusable poses
  // -------------------------

  /** Safe base posture. */
  public static final Pose STOW = new Pose(ElevatorLevel.STOW, WristPreset.STOW);

  // Coral approach / score
  public static Pose coralHover(ElevatorLevel level, boolean isAuto) {
    return switch (level) {
      case L1 -> new Pose(ElevatorLevel.L1, WristPreset.HOVER_L1);
      case L2 -> new Pose(ElevatorLevel.L2, WristPreset.HOVER_L2);
      case L3 -> new Pose(ElevatorLevel.L3, WristPreset.HOVER_L3);
      case L4 -> new Pose(ElevatorLevel.L4, isAuto ? WristPreset.AUTO_HOVER_L4 : WristPreset.HOVER_L4);
      default -> STOW;
    };
  }

  public static Pose coralDunk(ElevatorLevel level) {
    return switch (level) {
      case L1 -> new Pose(ElevatorLevel.L1, WristPreset.DUNK_L1);
      case L2 -> new Pose(ElevatorLevel.L2, WristPreset.DUNK_L2);
      case L3 -> new Pose(ElevatorLevel.L3, WristPreset.DUNK_L3);
      case L4 -> new Pose(ElevatorLevel.L4, WristPreset.DUNK_L4);
      default -> STOW;
    };
  }

  // Algae intake / travel / processor (no effector modes here—just posture)
  public static final Pose ALGAE_GROUND_INTAKE   = new Pose(ElevatorLevel.STOW,       WristPreset.GROUND_ALGAE);
  public static final Pose ALGAE_REEF_LOW_INTAKE = new Pose(ElevatorLevel.ALGAE_LOW,  WristPreset.REEF_ALGAE_LOW);
  public static final Pose ALGAE_REEF_HIGH_INTAKE= new Pose(ElevatorLevel.ALGAE_HIGH, WristPreset.REEF_ALGAE_HIGH);
  public static final Pose ALGAE_TRAVEL_HOLD     = new Pose(ElevatorLevel.STOW,       WristPreset.STOW);
  public static final Pose ALGAE_AT_PROCESSOR    = new Pose(ElevatorLevel.STOW,       WristPreset.PROCESSOR);
}
