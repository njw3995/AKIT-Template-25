package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d; // kept for parity (unused in elevator)
import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.signals.NeutralModeValue;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public ElevatorIOData data =
        new ElevatorIOData(
          false,  // motorConnected
          false,  // followerConnected
          false,  // tempFault
          false,  // followerTempFault
          false,  // zeroSwitchTriggered
          false,  // lowerLimitTriggered
          0.0,    // positionMeters
          0.0,    // velocityMetersPerSec
          0.0,    // appliedVolts
          0.0,    // torqueCurrentAmps
          0.0,    // supplyCurrentAmps
          0.0,    // elevatorPositionGoalMeters
          0.0,    // elevatorPositionSetpointMeters
          0.0,    // elevatorPositionErrorMeters
          0.0,    // tempCelsiusLeader
          0.0);   // tempCelsiusFollower
  }

  record ElevatorIOData(
      boolean motorConnected,
      boolean followerConnected,
      boolean tempFault,
      boolean followerTempFault,
      boolean zeroSwitchTriggered,
      boolean lowerLimitTriggered,
      double positionMeters,
      double velocityMetersPerSec,
      double appliedVolts,
      double torqueCurrentAmps,
      double supplyCurrentAmps,
      double elevatorPositionGoalMeters,
      double elevatorPositionSetpointMeters,
      double elevatorPositionErrorMeters,
      double tempCelsiusLeader,
      double tempCelsiusFollower) {}

  /**
   * Updates the inputs for the elevator subsystem.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Sets the voltage for the elevator.
   *
   * @param volts The voltage to set.
   */
  public default void setElevatorVoltage(double volts) {}

  /**
   * Sets the position goal for the elevator (meters).
   *
   * @param meters The position to set in meters.
   */
  public default void setElevatorPosition(double meters) {}

  /**
   * Slot0 gains for the elevator.
   */
  public default void updateSlot0ElevatorGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {}

  /**
   * Slot1 gains for the elevator (optional alt profile, e.g., stow).
   */
  public default void updateSlot1ElevatorGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {}

  /**
   * Motion constraints for the elevator.
   *
   * @param maxAcceleration The maximum acceleration (m/s^2).
   * @param cruisingVelocity The cruising velocity (m/s).
   */
  public default void updateElevatorConstraints(double maxAcceleration, double cruisingVelocity) {}

  /** Zero the elevator’s encoder to MIN height. */
  public default void zeroElevatorPosition() {}

  /** Optional hook to align controller toward max side before homing push. */
  public default void elevatorMax() {}

  /** Set neutral mode (Brake/Coast). */
  public default void setNeutralMode(NeutralModeValue mode) {}
}
