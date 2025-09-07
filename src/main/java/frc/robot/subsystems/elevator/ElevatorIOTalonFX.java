package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants.KrakenConstants;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {

  // Hardware
  private final TalonFX motor;
  private final TalonFX followerMotor;

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerTorqueCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Temperature> followerTemp;
  private StatusSignal<Double> positionSetpointRotations;
  private StatusSignal<Double> positionErrorRotations;

  private double positionGoalMeters;

  private MotionMagicVoltage positionVoltageRequest =
      new MotionMagicVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  public ElevatorIOTalonFX() {
    motor = new TalonFX(Constants.CanIDS.ELEVATOR_LEADER_CAN_ID, "Mechanisms");
    followerMotor = new TalonFX(Constants.CanIDS.ELEVATOR_FOLLOWER_CAN_ID, "Mechanisms");
    followerMotor.setControl(new Follower(motor.getDeviceID(), true));

    // Configure motor
    config.Slot0.kP = ElevatorConstants.GAINS.kP().get();
    config.Slot0.kD = ElevatorConstants.GAINS.kD().get();
    config.Slot0.kS = ElevatorConstants.GAINS.kS().get();
    config.Slot0.kV = ElevatorConstants.GAINS.kV().get();
    config.Slot0.kA = ElevatorConstants.GAINS.kA().get();
    config.Slot0.kG = ElevatorConstants.GAINS.kG().get();
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    config.Slot1.kP = ElevatorConstants.STOW_GAINS.kP().get();
    config.Slot1.kD = ElevatorConstants.STOW_GAINS.kD().get();
    config.Slot1.kS = ElevatorConstants.STOW_GAINS.kS().get();
    config.Slot1.kV = ElevatorConstants.STOW_GAINS.kV().get();
    config.Slot1.kA = ElevatorConstants.STOW_GAINS.kA().get();
    config.Slot1.kG = ElevatorConstants.STOW_GAINS.kG().get();
    config.Slot1.GravityType = GravityTypeValue.Elevator_Static;

    config.MotorOutput.NeutralMode =
        ElevatorConstants.IS_BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = ElevatorConstants.GEARING;
    config.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = ElevatorConstants.USE_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = ElevatorConstants.SUPPLY_LOWER_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime =
        ElevatorConstants.USE_SUPPLY_LOWER_CURRENT_LIMIT
            ? ElevatorConstants.SUPPLY_CURRENT_LIMIT_TIMEOUT
            : 0;
    config.MotorOutput.Inverted =
        ElevatorConstants.IS_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    torqueCurrent = motor.getTorqueCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    temp = motor.getDeviceTemp();
    followerAppliedVolts = followerMotor.getMotorVoltage();
    followerTorqueCurrent = followerMotor.getTorqueCurrent();
    followerSupplyCurrent = followerMotor.getSupplyCurrent();
    followerTemp = followerMotor.getDeviceTemp();
    positionGoalMeters = 0.0;
    positionSetpointRotations = motor.getClosedLoopReference();
    positionErrorRotations = motor.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity,
        appliedVolts,
        supplyCurrent,
        temp,
        followerAppliedVolts,
        followerTorqueCurrent,
        followerSupplyCurrent,
        followerTemp,
        positionSetpointRotations,
        positionErrorRotations);
    torqueCurrent.setUpdateFrequency(250);
    ParentDevice.optimizeBusUtilizationForAll(motor, followerMotor);

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        true,
        position,
        velocity,
        appliedVolts,
        torqueCurrent,
        supplyCurrent,
        temp,
        followerAppliedVolts,
        followerTorqueCurrent,
        followerSupplyCurrent,
        followerTemp);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.data =
        new ElevatorIOData(
            // Exclude torque-current b/c it's running at a much higher update rate
            BaseStatusSignal.isAllGood(position, velocity, appliedVolts, supplyCurrent, temp),
            BaseStatusSignal.isAllGood(
                followerAppliedVolts, followerTorqueCurrent, followerSupplyCurrent, followerTemp),
            position.getValueAsDouble() * ElevatorConstants.ROTATIONS_TO_METERS,
            velocity.getValueAsDouble() * ElevatorConstants.ROTATIONS_TO_METERS,
            appliedVolts.getValueAsDouble(),
            torqueCurrent.getValueAsDouble(),
            supplyCurrent.getValueAsDouble(),
            temp.getValueAsDouble(),
            followerAppliedVolts.getValueAsDouble(),
            followerTorqueCurrent.getValueAsDouble(),
            followerSupplyCurrent.getValueAsDouble(),
            followerTemp.getValueAsDouble(),
            positionGoalMeters,
            positionSetpointRotations.getValueAsDouble() * ElevatorConstants.ROTATIONS_TO_METERS,
            positionErrorRotations.getValueAsDouble() * ElevatorConstants.ROTATIONS_TO_METERS);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setPosition(double meters) {
    motor.setPosition(meters * ElevatorConstants.ROTATIONS_TO_METERS);
  }

  @Override
  public void setPositionGoal(double meters) {
    positionGoalMeters = meters;
    if (meters != 0.0) {
      motor.setControl(
          positionVoltageRequest
              .withPosition(meters * ElevatorConstants.ROTATIONS_TO_METERS)
              .withSlot(0));
    } else {
      motor.setControl(
          positionVoltageRequest
              .withPosition(meters * ElevatorConstants.ROTATIONS_TO_METERS)
              .withSlot(1));
    }
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void updateGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kG = kG;
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> followerMotor.getConfigurator().apply(config));
  }

  @Override
  public void updateConstraints(double maxAcceleration, double cruisingVelocity) {
    config.MotionMagic.MotionMagicAcceleration =
        maxAcceleration * ElevatorConstants.ROTATIONS_TO_METERS;
    config.MotionMagic.MotionMagicCruiseVelocity =
        cruisingVelocity * ElevatorConstants.ROTATIONS_TO_METERS;
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> followerMotor.getConfigurator().apply(config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () -> motor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast))
        .start();
  }
}
