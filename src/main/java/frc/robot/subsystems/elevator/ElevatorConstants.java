package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

public class ElevatorConstants {
  public static final double GEARING = 9;
  public static final double SPROCKET_PITCH_DIAMETER = 1.75666854583;
  public static final double ROTATIONS_TO_METERS =
      2
          * Math.PI
          * SPROCKET_PITCH_DIAMETER
          / GEARING
          * 0.0254; // ROTATIONS_TO_INCHES * INCHES_TO_METERS = ROTATIONS_TO_METERS

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

  public static final ElevatorParameters ELEVATOR_PARAMETERS;
  public static final Gains GAINS;
  public static final Constraints CONSTRAINTS;
  public static final Gains STOW_GAINS;
  public static final Constraints STOW_CONSTRAINTS;

  static {
    ELEVATOR_PARAMETERS =
        new ElevatorParameters(
            DCMotor.getKrakenX60Foc(2), 6.803886, 0.0, 1.43 + Units.inchesToMeters(0.5), 2);
    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
      default:
        GAINS =
            new Gains(
                new LoggedTunableNumber("Elevator/Gains/kP", 2.0),
                new LoggedTunableNumber("Elevator/Gains/kD", 0.1),
                new LoggedTunableNumber("Elevator/Gains/kS", 0.225),
                new LoggedTunableNumber("Elevator/Gains/kG", 0.075),
                new LoggedTunableNumber("Elevator/Gains/kV", 0.0),
                new LoggedTunableNumber("Elevator/Gains/kA", 0.0));
        CONSTRAINTS =
            new Constraints(
                new LoggedTunableNumber("Elevator/Max Acceleration", 16.0),
                new LoggedTunableNumber("Elevator/Cruising Velocity", 16.0),
                new LoggedTunableNumber("Elevator/Goal Tolerance", 0.02));
        STOW_GAINS =
            new Gains(
                new LoggedTunableNumber("Elevator/Gains/kP", 2.0),
                new LoggedTunableNumber("Elevator/Gains/kD", 0.1),
                new LoggedTunableNumber("Elevator/Gains/kS", 0.225),
                new LoggedTunableNumber("Elevator/Gains/kG", 0.075),
                new LoggedTunableNumber("Elevator/Gains/kV", 0.0),
                new LoggedTunableNumber("Elevator/Gains/kA", 0.0));
        STOW_CONSTRAINTS =
            new Constraints(
                new LoggedTunableNumber("Elevator/Max Acceleration", 16.0),
                new LoggedTunableNumber("Elevator/Cruising Velocity", 16.0),
                new LoggedTunableNumber("Elevator/Goal Tolerance", 0.02));
        break;
      case SIM:
        GAINS =
            new Gains(
                new LoggedTunableNumber("Elevator/Gains/kP", 20.0),
                new LoggedTunableNumber("Elevator/Gains/kD", 0.0),
                new LoggedTunableNumber("Elevator/Gains/kS", 0.0),
                new LoggedTunableNumber("Elevator/Gains/kG", 0.0),
                new LoggedTunableNumber("Elevator/Gains/kV", 0.0),
                new LoggedTunableNumber("Elevator/Gains/kA", 0.0));
        CONSTRAINTS =
            new Constraints(
                new LoggedTunableNumber("Elevator/Max Acceleration", 101.078594),
                new LoggedTunableNumber("Elevator/Cruising Velocity", 11.329982),
                new LoggedTunableNumber("Elevator/Goal Tolerance", 0.02));
        STOW_GAINS =
            new Gains(
                new LoggedTunableNumber("Elevator/Gains/kP", 20.0),
                new LoggedTunableNumber("Elevator/Gains/kD", 0.0),
                new LoggedTunableNumber("Elevator/Gains/kS", 0.0),
                new LoggedTunableNumber("Elevator/Gains/kG", 0.0),
                new LoggedTunableNumber("Elevator/Gains/kV", 0.0),
                new LoggedTunableNumber("Elevator/Gains/kA", 0.0));
        STOW_CONSTRAINTS =
            new Constraints(
                new LoggedTunableNumber("Elevator/Max Acceleration", 101.078594),
                new LoggedTunableNumber("Elevator/Cruising Velocity", 11.329982),
                new LoggedTunableNumber("Elevator/Goal Tolerance", 0.02));
        break;
    }
  }

  public static record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kG,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA) {}

  public static record Constraints(
      LoggedTunableNumber maxAccelerationMetersPerSecondSquared,
      LoggedTunableNumber cruisingVelocityMetersPerSecond,
      LoggedTunableNumber goalToleranceMeters) {}

  public static record ElevatorParameters(
      DCMotor ELEVATOR_MOTOR_CONFIG,
      double CARRIAGE_MASS_KG,
      double MIN_HEIGHT_METERS,
      double MAX_HEIGHT_METERS,
      int NUM_MOTORS) {}

  @RequiredArgsConstructor
  public static enum ElevatorPositions {
    STOW(0.0),
    CORAL_INTAKE(0.0),
    ALGAE_INTAKE(0.2161583093038944 + Units.inchesToMeters(1)),
    ALGAE_MID(0.7073684509805078),
    ALGAE_INTAKE_TOP(1.17 - Units.inchesToMeters(8)),
    ALGAE_INTAKE_BOT(0.79 - Units.inchesToMeters(8)),
    ASS_TOP(1.2),
    ASS_BOT(0.82),
    L1(0.11295250319916351),
    L2(0.37296301250898894),
    L3(0.7606347556550676 + Units.inchesToMeters(1.0)),
    L4(1.3864590139769697 + Units.inchesToMeters(0.5)),
    L4_PLUS(1.3864590139769697 + Units.inchesToMeters(2.0)),
    ALGAE_SCORE(1.3864590139769697 + Units.inchesToMeters(0.5));

    private final double position;

    public double getPosition() {
      return position;
    }
  }
}
