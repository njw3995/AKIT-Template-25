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
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOTalonFX implements ElevatorIO {

  // Hardware
  private final TalonFX motor;
  private final TalonFX follower;
  private final DigitalInput zeroSwitch;
  private final DigitalInput lowerLimit;

  // Config (start from constants’ MOTOR_CONFIG and keep a live copy for updates)
  private final TalonFXConfiguration liveCfg = new TalonFXConfiguration();

  // Status Signals
  private final StatusSignal<Angle> rotorPos;
  private final StatusSignal<AngularVelocity> rotorVel;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> motorTemp;
  private final StatusSignal<Temperature> followerTemp;
  private final StatusSignal<Boolean> leaderTempFault;
  private final StatusSignal<Boolean> followerTempFault;
  private final StatusSignal<Double> positionSetpointRot;
  private final StatusSignal<Double> positionErrorRot;

  private final MotionMagicVoltage positionVoltageRequest =
      new MotionMagicVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  public ElevatorIOTalonFX() {
    motor = new TalonFX(Constants.CanIDS.ELEVATOR_LEADER_CAN_ID, "Mechanisms");
    follower = new TalonFX(Constants.CanIDS.ELEVATOR_FOLLOWER_CAN_ID, "Mechanisms");
    zeroSwitch = new DigitalInput(Constants.DIOPorts.ELEVATOR_ZERO_LIMIT_DIO_PORT);
    lowerLimit = new DigitalInput(Constants.DIOPorts.ELEVATOR_LOWER_LIMIT_DIO_PORT);
    follower.setControl(new Follower(motor.getDeviceID(), true));

    // Apply baseline config from constants (parity with Wrist/EndEffector)
    tryUntilOk(5, () -> motor.getConfigurator().apply(ElevatorConstants.MOTOR_CONFIG, 0.25));
    tryUntilOk(5, () -> follower.getConfigurator().apply(ElevatorConstants.MOTOR_CONFIG, 0.25));

    rotorPos = motor.getPosition();
    rotorVel = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    torqueCurrent = motor.getTorqueCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    motorTemp = motor.getDeviceTemp();
    followerTemp = follower.getDeviceTemp();
    leaderTempFault = motor.getFault_DeviceTemp();
    followerTempFault = follower.getFault_DeviceTemp();
    positionSetpointRot = motor.getClosedLoopReference();
    positionErrorRot = motor.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        rotorPos,
        rotorVel,
        appliedVolts,
        supplyCurrent,
        motorTemp,
        followerTemp,
        positionSetpointRot,
        positionErrorRot,
        leaderTempFault,
        followerTempFault);
    ParentDevice.optimizeBusUtilizationForAll(motor, follower);

    PhoenixUtil.registerSignals(
        true,
        rotorPos,
        rotorVel,
        appliedVolts,
        torqueCurrent,
        supplyCurrent,
        motorTemp,
        followerTemp,
        leaderTempFault,
        followerTempFault);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    final double rot = rotorPos.getValueAsDouble();
    final double rps = rotorVel.getValueAsDouble();

    inputs.data =
        new ElevatorIOData(
            BaseStatusSignal.isAllGood(rotorPos, rotorVel, appliedVolts, supplyCurrent, motorTemp),
            BaseStatusSignal.isAllGood(followerTemp),
            leaderTempFault.getValue(),
            followerTempFault.getValue(),
            !zeroSwitch.get(),
            !lowerLimit.get(),
            rot * ElevatorConstants.ROTATIONS_TO_METERS,
            rps * ElevatorConstants.ROTATIONS_TO_METERS,
            appliedVolts.getValueAsDouble(),
            torqueCurrent.getValueAsDouble(),
            supplyCurrent.getValueAsDouble(),
            /* goal m */ positionSetpointRot.getValueAsDouble()
                * ElevatorConstants.ROTATIONS_TO_METERS,
            /* sp m   */ positionSetpointRot.getValueAsDouble()
                * ElevatorConstants.ROTATIONS_TO_METERS,
            /* err m  */ positionErrorRot.getValueAsDouble()
                * ElevatorConstants.ROTATIONS_TO_METERS,
            motorTemp.getValueAsDouble(),
            followerTemp.getValueAsDouble());
  }

  @Override
  public void setElevatorVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setElevatorPosition(double meters) {
    final double rotations = meters / ElevatorConstants.ROTATIONS_TO_METERS;
    final int slot = rotations > rotorPos.getValueAsDouble() ? 0 : 1;
    motor.setControl(positionVoltageRequest.withPosition(rotations).withSlot(slot));
    Logger.recordOutput("Elevator/ActiveSlot", slot);
  }

  @Override
  public void updateSlot0ElevatorGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {
    liveCfg.Slot0.kP = kP;
    liveCfg.Slot0.kD = kD;
    liveCfg.Slot0.kS = kS;
    liveCfg.Slot0.kV = kV;
    liveCfg.Slot0.kA = kA;
    liveCfg.Slot0.kG = kG;
    tryUntilOk(5, () -> motor.getConfigurator().apply(liveCfg));
    tryUntilOk(5, () -> follower.getConfigurator().apply(liveCfg));
  }

  @Override
  public void updateSlot1ElevatorGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {
    liveCfg.Slot1.kP = kP;
    liveCfg.Slot1.kD = kD;
    liveCfg.Slot1.kS = kS;
    liveCfg.Slot1.kV = kV;
    liveCfg.Slot1.kA = kA;
    liveCfg.Slot1.kG = kG;
    tryUntilOk(5, () -> motor.getConfigurator().apply(liveCfg));
    tryUntilOk(5, () -> follower.getConfigurator().apply(liveCfg));
  }

  @Override
  public void updateElevatorConstraints(double maxAcceleration, double cruisingVelocity) {
    // meters/sec -> mech rotations/sec (NOT rotor rps)
    final double cruiseRps = cruisingVelocity / ElevatorConstants.SPROCKET_CIRCUMFERENCE_METERS;
    final double accelRps2 = maxAcceleration / ElevatorConstants.SPROCKET_CIRCUMFERENCE_METERS;
    liveCfg.MotionMagic.MotionMagicAcceleration = accelRps2;
    liveCfg.MotionMagic.MotionMagicCruiseVelocity = cruiseRps;
    tryUntilOk(5, () -> motor.getConfigurator().apply(liveCfg));
    tryUntilOk(5, () -> follower.getConfigurator().apply(liveCfg));
  }

  @Override
  public void zeroElevatorPosition() {
    // set to MIN height in rotor rotations space
    final double minRot =
        ElevatorConstants.MIN_HEIGHT_METERS / ElevatorConstants.ROTATIONS_TO_METERS;
    motor.setPosition(minRot);
  }

  @Override
  public void elevatorMax() {
    // Optional hook before homing push; no-op by default
  }

  @Override
  public void setNeutralMode(NeutralModeValue mode) {
    motor.setNeutralMode(mode);
    follower.setNeutralMode(mode);
  }
}
