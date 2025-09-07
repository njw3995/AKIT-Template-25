package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public ElevatorIOData data =
        new ElevatorIOData(
            false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  record ElevatorIOData(
      boolean motorConnected,
      boolean followerConnected,
      double positionMeters,
      double velocityMetersPerSec,
      double appliedVolts,
      double torqueCurrentAmps,
      double supplyCurrentAmps,
      double tempCelsius,
      double followerAppliedVolts,
      double followerTorqueCurrentAmps,
      double followerSupplyCurrentAmps,
      double followerTempCelsius,
      double positionGoalMeters,
      double positionSetpointMeters,
      double positionErrorMeters) {}

  /**
   * Updates the inputs for the elevator.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Sets the voltage for the elevator.
   *
   * @param volts The voltage to set.
   */
  public default void setVoltage(double volts) {}

  /** Stops the motors on the elevator. */
  public default void stop() {}

  /**
   * Sets the position for the elevator.
   *
   * @param meters The position to set in meters.
   */
  public default void setPosition(double meters) {}

  /**
   * Sets the position goal for the elevator.
   *
   * @param meters The position goal to set in meters.
   */
  public default void setPositionGoal(double meters) {}

  /**
   * Sets the gains for the elevator.
   *
   * @param kP The proportional gain.
   * @param kD The derivative gain.
   * @param kS The static gain.
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   * @param kG The gravity gain.
   */
  public default void updateGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {}

  /**
   * Sets the constraints for the elevator.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param cruisingVelocity The cruising velocity.
   */
  public default void updateConstraints(double maxAcceleration, double cruisingVelocity) {}

  default void setBrakeMode(boolean enabled) {}
}
