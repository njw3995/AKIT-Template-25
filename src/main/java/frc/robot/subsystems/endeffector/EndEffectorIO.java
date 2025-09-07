package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  class EndEffectorIOInputs {
    public EndEffectorIOData data =
        new EndEffectorIOData(0, 0, 0, 0, 0, 0, false, false, false, false);
  }

  record EndEffectorIOData(
      double positionRads,
      double velocityRadsPerSec,
      double appliedVoltage,
      double supplyCurrentAmps,
      double torqueCurrentAmps,
      double tempCelsius,
      boolean tempFault,
      boolean intakeBeamBreakDetected,
      boolean placementBeamBreakDetected,
      boolean connected) {}

  default void updateInputs(EndEffectorIOInputs inputs) {}

  default void runVolts(double volts) {}

  default void runTorqueCurrent(double amps) {}

  default void stop() {}

  default void setStatorCurrentLimit(double currentLimit) {}

  default void setBrakeMode(boolean enabled) {}
}
