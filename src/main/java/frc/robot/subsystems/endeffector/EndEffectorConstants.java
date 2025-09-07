package frc.robot.subsystems.endeffector;

public final class EndEffectorConstants {
  public static final double GEARING = 5;

  public static final double HAS_ALGAE_CURRENT =
      10; // The minimum current the motor is running at to assume there's an algae
  public static final double HAS_ALGAE_MAX_VELOCITY =
      50; // the maximum velocity the motor could be spinning at for there to be algae

  public static final boolean IS_INVERTED = false;
  public static final int SUPPLY_CURRENT_LIMIT = 40;
  public static final boolean USE_SUPPLY_CURRENT_LIMIT = true;
  public static final int STATOR_CURRENT_LIMIT = 80;
  public static final boolean USE_STATOR_CURRENT_LIMIT = true;
  public static final boolean IS_BRAKE = false;
}
