package frc.robot.subsystems.endeffector;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants.KrakenConstants;
import frc.robot.util.PhoenixUtil;

public class EndEffectorIOTalonFX implements EndEffectorIO {
  private final TalonFX motor;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;
  private final StatusSignal<Boolean> tempFault;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC torqueCurrentOut = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0);
  private final NeutralOut neutralOut = new NeutralOut();

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final DigitalInput intakeCoralBeakBreak;
  private final DigitalInput placementCoralBeamBreak;

  public EndEffectorIOTalonFX() {
    motor = new TalonFX(Constants.CanIDS.END_EFFECTOR_CAN_ID, "Mechanisms");
    intakeCoralBeakBreak =
        new DigitalInput(Constants.DIOPorts.END_EFFECTOR_INTAKE_CORAL_BEAM_BREAK_DIO_PORT);
    placementCoralBeamBreak =
        new DigitalInput(Constants.DIOPorts.END_EFFECTOR_PLACEMENT_CORAL_BEAM_BREAK_DIO_PORT);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        EndEffectorConstants.IS_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode =
        EndEffectorConstants.IS_BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimit = EndEffectorConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = EndEffectorConstants.USE_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimit = EndEffectorConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = EndEffectorConstants.USE_STATOR_CURRENT_LIMIT;

    config.Feedback.SensorToMechanismRatio = EndEffectorConstants.GEARING;
    config.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

    tryUntilOk(5, () -> motor.getConfigurator().apply(config));

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVoltage = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    torqueCurrent = motor.getTorqueCurrent();
    tempCelsius = motor.getDeviceTemp();
    tempFault = motor.getFault_DeviceTemp();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius,
                tempFault));
    tryUntilOk(5, () -> motor.optimizeBusUtilization(0, 1.0));

    PhoenixUtil.registerSignals(
        new CANBus("Mechanisms").isNetworkFD(),
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius,
        tempFault);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.data =
        new EndEffectorIOData(
            Units.rotationsToRadians(position.getValueAsDouble())
                / EndEffectorConstants
                    .GEARING, // Check if setting ratio above already implicity divides by gearing
            Units.rotationsToRadians(velocity.getValueAsDouble())
                / EndEffectorConstants
                    .GEARING, // Check if setting ratio above already implicity divides by gearing
            appliedVoltage.getValueAsDouble(),
            supplyCurrent.getValueAsDouble(),
            torqueCurrent.getValueAsDouble(),
            tempCelsius.getValueAsDouble(),
            tempFault.getValue(),
            !intakeCoralBeakBreak.get(),
            !placementCoralBeamBreak.get(),
            BaseStatusSignal.isAllGood(
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius,
                tempFault));
  }

  @Override
  public void runVolts(double volts) {
    motor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runTorqueCurrent(double amps) {
    motor.setControl(torqueCurrentOut.withOutput(amps));
  }

  @Override
  public void stop() {
    motor.setControl(neutralOut);
  }

  @Override
  public void setStatorCurrentLimit(double currentLimit) {
    new Thread(
        () -> {
          config.withCurrentLimits(config.CurrentLimits.withStatorCurrentLimit(currentLimit));
          tryUntilOk(5, () -> motor.getConfigurator().apply(config));
        });
  }
}
