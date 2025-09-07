package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  class WristIOInputs {
    public WristIOData data =
        new WristIOData(
            false,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            new Rotation2d(),
            new Rotation2d(),
            new Rotation2d(),
            0.0);
  }

  record WristIOData(
      boolean motorConnected,
      double positionRad,
      double velocityRadPerSec,
      double appliedVolts,
      double torqueCurrentAmps,
      double supplyCurrentAmps,
      Rotation2d wristPositionGoal,
      Rotation2d wristPositionSetpoint,
      Rotation2d wristPositionError,
      double tempCelsius) {}

  /**
   * Updates the inputs for the wrist subsystem.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(WristIOInputs inputs) {}

  /**
   * Sets the voltage for the wrist.
   *
   * @param volts The voltage to set.
   */
  public default void setWristVoltage(double volts) {}

  /**
   * Sets the position goal for the wrist.
   *
   * @param rotation The position to set in radians.
   */
  public default void setWristPosition(Rotation2d rotation) {}

  /**
   * Sets the gains for the wrist.
   *
   * @param kP The proportional gain.
   * @param kD The derivative gain.
   * @param kS The static gain.
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   * @param kG The gravity gain.
   */
  public default void updateSlot0WristGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {}

  public default void updateSlot1WristGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {}

  /**
   * Sets the constraints for the wrist.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param cruisingVelocity The cruising velocity.
   */
  public default void updateWristConstraints(double maxAcceleration, double cruisingVelocity) {}

  public default void zeroWristPosition() {}

  public default void wristMax() {}
}
