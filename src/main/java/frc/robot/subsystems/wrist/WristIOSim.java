package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class WristIOSim implements WristIO {
  private final SingleJointedArmSim wristSim;

  private double wristAppliedVolts;

  private final ProfiledPIDController feedback;
  private ArmFeedforward feedforward;

  private boolean wristClosedLoop;

  public WristIOSim() {
    wristSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                WristConstants.MOTOR_MODEL, // motor
                0.004, // moment of inertia (kg*m^2) – tune as needed
                WristConstants.GEARING),
            WristConstants.MOTOR_MODEL,
            WristConstants.GEARING,
            WristConstants.LENGTH_METERS,
            WristConstants.MIN_ANGLE.getRadians(),
            WristConstants.MAX_ANGLE.getRadians(),
            true,
            WristConstants.MIN_ANGLE.getRadians());

    wristAppliedVolts = 0.0;

    feedback =
        new ProfiledPIDController(
            WristConstants.S0_kP,
            0.0,
            WristConstants.S0_kD,
            new Constraints(WristConstants.CRUISE_VEL_RPS, WristConstants.MAX_ACCEL_RPS2));

    feedforward =
        new ArmFeedforward(WristConstants.S0_kS, WristConstants.S0_kG, WristConstants.S0_kV);

    wristClosedLoop = true;
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    if (wristClosedLoop)
      wristAppliedVolts =
          feedback.calculate(wristSim.getAngleRads())
              + feedforward.calculate(
                  feedback.getSetpoint().position, feedback.getSetpoint().velocity);

    wristAppliedVolts = MathUtil.clamp(wristAppliedVolts, -12.0, 12.0);

    wristSim.setInputVoltage(wristAppliedVolts);
    wristSim.update(Constants.loopPeriodSecs);

    inputs.data =
        new WristIOData(
            // Exclude torque-current b/c it's running at a much higher update rate
            true,
            false,
            wristSim.getAngleRads(),
            wristSim.getVelocityRadPerSec(),
            wristAppliedVolts,
            wristSim.getCurrentDrawAmps(),
            wristSim.getCurrentDrawAmps(),
            Rotation2d.fromRadians(feedback.getGoal().position),
            Rotation2d.fromRadians(feedback.getSetpoint().position),
            Rotation2d.fromRadians(feedback.getPositionError()),
            0);
  }

  @Override
  public void setWristPosition(Rotation2d position) {
    wristClosedLoop = true;
    feedback.setGoal(position.getRadians());
  }

  @Override
  public void setWristVoltage(double wristAppliedVolts) {
    wristClosedLoop = false;
    this.wristAppliedVolts = wristAppliedVolts;
  }

  @Override
  public void updateSlot0WristGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {
    feedback.setPID(kP, 0, kD);
    feedforward = new ArmFeedforward(kS, kG, kV);
  }

  @Override
  public void updateWristConstraints(
      double maxAccelerationRadiansPerSecondSquared, double cruisingVelocityRadiansPerSecond) {
    feedback.setConstraints(
        new Constraints(cruisingVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared));
  }
}
