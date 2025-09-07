package frc.robot.subsystems.elevator;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim sim;

  private final ProfiledPIDController feedback;
  private ElevatorFeedforward feedforward;

  private double appliedVolts;
  private boolean isClosedLoop;

  public ElevatorIOSim() {
    sim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                ElevatorConstants.ELEVATOR_PARAMETERS.ELEVATOR_MOTOR_CONFIG(),
                4,
                ElevatorConstants.SPROCKET_PITCH_DIAMETER * 0.0254 / 2,
                ElevatorConstants.GEARING),
            ElevatorConstants.ELEVATOR_PARAMETERS.ELEVATOR_MOTOR_CONFIG(),
            ElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS(),
            ElevatorConstants.ELEVATOR_PARAMETERS.MAX_HEIGHT_METERS(),
            true,
            ElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS());

    feedback =
        new ProfiledPIDController(
            ElevatorConstants.GAINS.kP().get(),
            0,
            ElevatorConstants.GAINS.kD().get(),
            new Constraints(
                ElevatorConstants.CONSTRAINTS.cruisingVelocityMetersPerSecond().get(),
                ElevatorConstants.CONSTRAINTS.maxAccelerationMetersPerSecondSquared().get()));

    feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.GAINS.kS().get(),
            ElevatorConstants.GAINS.kG().get(),
            ElevatorConstants.GAINS.kV().get());

    appliedVolts = 0.0;
    isClosedLoop = true;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (isClosedLoop) {
      appliedVolts =
          feedback.calculate(sim.getPositionMeters())
              + feedforward.calculate(feedback.getSetpoint().position);
    }
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);

    inputs.data =
        new ElevatorIOData(
            true,
            true,
            sim.getPositionMeters(),
            sim.getVelocityMetersPerSecond(),
            appliedVolts,
            Math.copySign(sim.getCurrentDrawAmps(), appliedVolts),
            Math.copySign(sim.getCurrentDrawAmps(), appliedVolts),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            feedback.getGoal().position,
            feedback.getSetpoint().position,
            feedback.getPositionError());
  }

  @Override
  public void setVoltage(double volts) {
    isClosedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void stop() {
    isClosedLoop = false;
    appliedVolts = 0;
  }

  @Override
  public void setPosition(double position) {
    sim.setState(position, 0);
  }

  @Override
  public void setPositionGoal(double position) {
    isClosedLoop = true;
    feedback.setGoal(position);
  }

  @Override
  public void updateGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    feedback.setPID(kP, 0, kD);
    feedforward = new ElevatorFeedforward(kS, kG, kV);
  }

  @Override
  public void updateConstraints(double maxAcceleration, double cruisingVelocity) {
    feedback.setConstraints(new Constraints(cruisingVelocity, maxAcceleration));
  }
}
