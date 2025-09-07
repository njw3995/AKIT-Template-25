package frc.robot.subsystems.wrist;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants.KrakenConstants;
import frc.robot.util.PhoenixUtil;

public class WristIOTalonFX implements WristIO {
  private final TalonFX motor;

  private final TalonFXConfiguration config;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Double> setpointRotations;
  private final StatusSignal<Double> setpointErrorRotations;

  private Rotation2d wristPositionGoal;

  private final DynamicMotionMagicVoltage positionControlRequest;
  private final VoltageOut voltageRequest;

  public WristIOTalonFX() {
    motor = new TalonFX(Constants.CanIDS.WRIST_CAN_ID, "Mechanisms");

    config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = WristConstants.WRIST_PARAMETERS.GEARING();
    config.MotorOutput.NeutralMode =
        WristConstants.IS_BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

    config.CurrentLimits.SupplyCurrentLimit = WristConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = WristConstants.USE_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = WristConstants.SUPPLY_LOWER_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime =
        WristConstants.USE_SUPPLY_LOWER_CURRENT_LIMIT
            ? WristConstants.SUPPLY_CURRENT_LIMIT_TIMEOUT
            : 0;
    config.MotorOutput.Inverted =
        WristConstants.IS_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Slot0.kP = WristConstants.WITHOUT_ALGAE_GAINS.kP().get();
    config.Slot0.kD = WristConstants.WITHOUT_ALGAE_GAINS.kD().get();
    config.Slot0.kS = WristConstants.WITHOUT_ALGAE_GAINS.kS().get();
    config.Slot0.kV = WristConstants.WITHOUT_ALGAE_GAINS.kV().get();
    config.Slot0.kA = WristConstants.WITHOUT_ALGAE_GAINS.kA().get();
    config.Slot1.kP = WristConstants.WITH_ALGAE_GAINS.kP().get();
    config.Slot1.kD = WristConstants.WITH_ALGAE_GAINS.kD().get();
    config.Slot1.kS = WristConstants.WITH_ALGAE_GAINS.kS().get();
    config.Slot1.kV = WristConstants.WITH_ALGAE_GAINS.kV().get();
    config.Slot1.kA = WristConstants.WITH_ALGAE_GAINS.kA().get();

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        WristConstants.WRIST_PARAMETERS.MAX_ANGLE().getRotations();
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        WristConstants.WRIST_PARAMETERS.MIN_ANGLE().getRotations();
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.MotionMagic.MotionMagicAcceleration =
        WristConstants.CONSTRAINTS.MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED().get();
    config.MotionMagic.MotionMagicCruiseVelocity =
        WristConstants.CONSTRAINTS.CRUISING_VELOCITY_ROTATIONS_PER_SECOND().get();

    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    torqueCurrent = motor.getTorqueCurrent();
    temp = motor.getDeviceTemp();
    wristPositionGoal = new Rotation2d();
    setpointRotations = motor.getClosedLoopReference();
    setpointErrorRotations = motor.getClosedLoopError();

    positionControlRequest =
        new DynamicMotionMagicVoltage(
            0,
            WristConstants.CONSTRAINTS.CRUISING_VELOCITY_ROTATIONS_PER_SECOND().get(),
            WristConstants.CONSTRAINTS.MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED().get(),
            0);
    voltageRequest = new VoltageOut(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity,
        appliedVolts,
        supplyCurrent,
        torqueCurrent,
        temp,
        setpointRotations,
        setpointErrorRotations);

    motor.optimizeBusUtilization();

    motor.setPosition(WristConstants.WRIST_PARAMETERS.MIN_ANGLE().getRotations());

    PhoenixUtil.registerSignals(
        false,
        position,
        velocity,
        appliedVolts,
        supplyCurrent,
        torqueCurrent,
        temp,
        setpointRotations,
        setpointErrorRotations);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.data =
        new WristIOData(
            // Exclude torque-current b/c it's running at a much higher update rate
            BaseStatusSignal.isAllGood(position, velocity, appliedVolts, supplyCurrent, temp),
            Units.rotationsToRadians(position.getValueAsDouble()),
            Units.rotationsToRadians(velocity.getValueAsDouble()),
            appliedVolts.getValueAsDouble(),
            torqueCurrent.getValueAsDouble(),
            supplyCurrent.getValueAsDouble(),
            wristPositionGoal,
            Rotation2d.fromRotations(setpointRotations.getValueAsDouble()),
            Rotation2d.fromRotations(setpointErrorRotations.getValueAsDouble()),
            temp.getValueAsDouble());
  }

  @Override
  public void setWristVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setWristPosition(Rotation2d position) {
    wristPositionGoal = position;
    motor.setControl(
        positionControlRequest.withPosition(position.getRotations()).withEnableFOC(true));
  }

  @Override
  public void updateSlot0WristGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateSlot1WristGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateWristConstraints(double maxAcceleration, double maxVelocity) {
    config.MotionMagic.MotionMagicAcceleration = maxAcceleration;
    config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void zeroWristPosition() {
    motor.setPosition(WristConstants.WRIST_PARAMETERS.MIN_ANGLE().getRotations());
  }

  @Override
  public void wristMax() {
    motor.setPosition(WristConstants.WRIST_PARAMETERS.MAX_ANGLE().getRotations());
  }
}
