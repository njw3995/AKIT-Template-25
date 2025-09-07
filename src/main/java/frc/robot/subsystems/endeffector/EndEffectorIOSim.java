package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class EndEffectorIOSim implements EndEffectorIO {
  private final DCMotorSim motor;
  private final DCMotor gearbox;
  private double appliedVoltage = 0.0;

  public EndEffectorIOSim() {
    gearbox = DCMotor.getKrakenX60Foc(1);
    motor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 0.01, EndEffectorConstants.GEARING),
            gearbox); // idk moi
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0.0);
    }

    motor.update(Constants.loopPeriodSecs);
    inputs.data =
        new EndEffectorIOData(
            motor.getAngularPositionRad(),
            motor.getAngularVelocityRadPerSec(),
            appliedVoltage,
            motor.getCurrentDrawAmps(),
            gearbox.getCurrent(motor.getAngularVelocityRadPerSec(), appliedVoltage),
            0.0,
            false,
            true, // Find a better way... Maybe just a toggle on smart dashboard for testing
            // algortithm.
            true, // Find a better way... Maybe just a toggle on smart dashboard for testing
            // algortithm.
            true);
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    motor.setInputVoltage(appliedVoltage);
  }

  @Override
  public void runTorqueCurrent(double amps) {
    runVolts(gearbox.getVoltage(gearbox.getTorque(amps), motor.getAngularVelocityRadPerSec()));
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
