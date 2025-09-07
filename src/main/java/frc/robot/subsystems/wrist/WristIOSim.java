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
                WristConstants.WRIST_PARAMETERS.MOTOR_CONFIG(),
                0.004,
                WristConstants.WRIST_PARAMETERS.GEARING()),
            WristConstants.WRIST_PARAMETERS.MOTOR_CONFIG(),
            WristConstants.WRIST_PARAMETERS.GEARING(),
            WristConstants.WRIST_PARAMETERS.LENGTH_METERS(),
            WristConstants.WRIST_PARAMETERS.MIN_ANGLE().getRadians(),
            WristConstants.WRIST_PARAMETERS.MAX_ANGLE().getRadians(),
            true,
            WristConstants.WRIST_PARAMETERS.MIN_ANGLE().getRadians());

    wristAppliedVolts = 0.0;

    feedback =
        new ProfiledPIDController(
            WristConstants.WITHOUT_ALGAE_GAINS.kP().get(),
            0.0,
            WristConstants.WITHOUT_ALGAE_GAINS.kD().get(),
            new Constraints(
                WristConstants.CONSTRAINTS.CRUISING_VELOCITY_ROTATIONS_PER_SECOND().get(),
                WristConstants.CONSTRAINTS.MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED().get()));
    feedforward =
        new ArmFeedforward(
            WristConstants.WITHOUT_ALGAE_GAINS.kS().get(),
            WristConstants.WITHOUT_ALGAE_GAINS.kV().get(),
            WristConstants.WITHOUT_ALGAE_GAINS.kA().get());

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
