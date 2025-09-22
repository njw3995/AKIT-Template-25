package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
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
    final double drumRadiusMeters = (ElevatorConstants.SPROCKET_PITCH_DIAMETER * 0.0254) / 2.0;

    sim = new ElevatorSim(
        LinearSystemId.createElevatorSystem(
            ElevatorConstants.MOTOR_MODEL,
            ElevatorConstants.CARRIAGE_MASS_KG,
            drumRadiusMeters,
            ElevatorConstants.GEARING),
        ElevatorConstants.MOTOR_MODEL,
        ElevatorConstants.MIN_HEIGHT_METERS,
        ElevatorConstants.MAX_HEIGHT_METERS,
        true,
        ElevatorConstants.MIN_HEIGHT_METERS);

    feedback = new ProfiledPIDController(
        ElevatorConstants.T_S0_kP.get(),
        0.0,
        ElevatorConstants.T_S0_kD.get(),
        new Constraints(
            ElevatorConstants.T_CRUISE_VEL_MPS.get(),
            ElevatorConstants.T_MAX_ACCEL_MPS2.get()));

    feedforward = new ElevatorFeedforward(
        ElevatorConstants.T_S0_kS.get(),
        ElevatorConstants.T_S0_kG.get(),
        ElevatorConstants.T_S0_kV.get());

    appliedVolts = 0.0;
    isClosedLoop = true;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // 20ms step
    if (isClosedLoop) {
      appliedVolts =
          feedback.calculate(sim.getPositionMeters())
              + feedforward.calculate(feedback.getSetpoint().position);
    }
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.data =
        new ElevatorIOData(
            true,  // motorConnected
            true,  // followerConnected
            false, // tempFault
            false,
            false,
            false,
            sim.getPositionMeters(),
            sim.getVelocityMetersPerSecond(),
            appliedVolts,
            Math.copySign(sim.getCurrentDrawAmps(), appliedVolts),
            Math.copySign(sim.getCurrentDrawAmps(), appliedVolts),
            feedback.getGoal().position,
            feedback.getSetpoint().position,
            feedback.getPositionError(),
            0.0,
            0.0);
  }

  @Override
  public void setElevatorVoltage(double volts) {
    isClosedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void setElevatorPosition(double meters) {
    isClosedLoop = true;
    feedback.setGoal(meters);
  }

  @Override
  public void updateSlot0ElevatorGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    feedback.setPID(kP, 0.0, kD);
    feedforward = new ElevatorFeedforward(kS, kG, kV);
  }

  @Override
  public void updateSlot1ElevatorGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    // SIM doesn’t use Slot1; left as no-op for parity.
  }

  @Override
  public void updateElevatorConstraints(double maxAcceleration, double cruisingVelocity) {
    feedback.setConstraints(new Constraints(cruisingVelocity, maxAcceleration));
  }

  @Override
  public void zeroElevatorPosition() {
    sim.setState(ElevatorConstants.MIN_HEIGHT_METERS, 0.0);
  }

  @Override
  public void elevatorMax() {
    // optional hook; no-op in sim
  }

  @Override
  public void setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue mode) {
    // sim no-op
  }
}
