package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

public class WristConstants {

  public static final boolean IS_INVERTED = false;
  public static final int SUPPLY_CURRENT_LIMIT = 80;
  public static final boolean USE_SUPPLY_CURRENT_LIMIT = true;
  public static final int SUPPLY_LOWER_CURRENT_LIMIT = 40;
  public static final double SUPPLY_CURRENT_LIMIT_TIMEOUT = 1.5;
  public static final boolean USE_SUPPLY_LOWER_CURRENT_LIMIT =
      true; // Default true, should probably remain that way?
  public static final int STATOR_CURRENT_LIMIT = 100;
  public static final boolean USE_STATOR_CURRENT_LIMIT = true;
  public static final boolean IS_BRAKE = true;

  public static final WristParameters WRIST_PARAMETERS;
  public static final Gains WITHOUT_ALGAE_GAINS;
  public static final Gains WITH_ALGAE_GAINS;
  public static final Constraints CONSTRAINTS;

  static {
    WRIST_PARAMETERS =
        new WristParameters(
            DCMotor.getKrakenX60Foc(1),
            Rotation2d.fromDegrees(-77.0),
            Rotation2d.fromDegrees(75.0),
            1,
            90.0,
            0.5);
    WITHOUT_ALGAE_GAINS =
        new Gains(
            new LoggedTunableNumber("Wrist/WithoutAlgae/kP", 125),
            new LoggedTunableNumber("Wrist/WithoutAlgae/kD", 0),
            new LoggedTunableNumber("Wrist/WithoutAlgae/kS", 0.24274),
            new LoggedTunableNumber("Wrist/WithoutAlgae/kG", 0.66177),
            new LoggedTunableNumber("Wrist/WithoutAlgae/kV", 0.0),
            new LoggedTunableNumber("Wrist/WithoutAlgae/kA", 0.0));
    WITH_ALGAE_GAINS =
        new Gains(
            new LoggedTunableNumber("Wrist/WithAlgae/kP", 125),
            new LoggedTunableNumber("Wrist/WithAlgae/kD", 0),
            new LoggedTunableNumber("Wrist/WithAlgae/kS", 0.65347),
            new LoggedTunableNumber("Wrist/WithAlgae/kG", 2.0762),
            new LoggedTunableNumber("Wrist/WithAlgae/kV", 0.0),
            new LoggedTunableNumber("Wrist/WithAlgae/kA", 0.0));
    CONSTRAINTS =
        new Constraints(
            new LoggedTunableNumber("Wrist/MaxAcceleration", 2.0),
            new LoggedTunableNumber("Wrist/CruisingVelocity", 5.0),
            new LoggedTunableNumber("Wrist/GoalTolerance", Units.degreesToRadians(1.5)));
  }

  public static record WristParameters(
      DCMotor MOTOR_CONFIG,
      Rotation2d MIN_ANGLE,
      Rotation2d MAX_ANGLE,
      int NUM_MOTORS,
      double GEARING,
      double LENGTH_METERS) {}

  public static record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kG,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA) {}

  public static record Constraints(
      LoggedTunableNumber MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED,
      LoggedTunableNumber CRUISING_VELOCITY_ROTATIONS_PER_SECOND,
      LoggedTunableNumber GOAL_TOLERANCE_RADIANS) {}

  @RequiredArgsConstructor
  public static enum WristState {
    STOW_UP(Rotation2d.fromDegrees(75)),
    PROCESSOR(Rotation2d.fromDegrees(-61.279296875 + 20)),
    REEF_INTAKE(Rotation2d.fromDegrees(-61.279296875 + 15)),
    INTAKE_OUT_LINE(Rotation2d.fromDegrees(-61)),
    FLOOR_INTAKE(Rotation2d.fromDegrees(-68.5 - 5)),
    STOW_LINE(Rotation2d.fromDegrees(-75)),
    STOW_DOWN(Rotation2d.fromDegrees(-77));

    private final Rotation2d angle;

    public Rotation2d getAngle() {
      return angle;
    }
  }
}
