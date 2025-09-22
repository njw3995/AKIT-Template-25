package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

/** All values live here; IO just applies MOTOR_CONFIG and uses the doubles. */
public final class ElevatorConstants {
  private ElevatorConstants() {}

  // ---------- Mechanism / units ----------
  /** Motor-to-sprocket ratio (sensor-to-mechanism ratio in Phoenix). */
  public static final double GEARING = 9.0;

  /** Sprocket pitch diameter (inches). DO NOT CHANGE NAME (validated with ROTATIONS_TO_METERS). */
  public static final double SPROCKET_PITCH_DIAMETER = 1.75666854583;

  /** Linear travel per *rotor* rotation (meters/rot): (π * D) / GEARING. */
  public static final double ROTATIONS_TO_METERS =
      (Math.PI * SPROCKET_PITCH_DIAMETER * Units.inchesToMeters(1.0)) / GEARING;

  /** Sprocket circumference (meters) — mechanism rotation distance (not divided by gearing). */
  public static final double SPROCKET_CIRCUMFERENCE_METERS =
      Math.PI * SPROCKET_PITCH_DIAMETER * Units.inchesToMeters(1.0);

  /** Soft limits (meters). */
  public static final double MIN_HEIGHT_METERS = 0.0;
  public static final double MAX_HEIGHT_METERS = 1.43 + Units.inchesToMeters(0.5);

  /** Sim/feeds helpers. */
  public static final int NUM_MOTORS = 2;
  public static final double CARRIAGE_MASS_KG = 6.803886;
  public static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX60Foc(NUM_MOTORS);

  // ---------- Electrical limits / mode ----------
  public static final boolean IS_BRAKE = true;
  /** true -> CCW Positive, false -> CW Positive (matches Wrist/EndEffector convention). */
  public static final boolean IS_INVERTED = false;

  public static final int SUPPLY_CURRENT_LIMIT = 80;
  public static final boolean USE_SUPPLY_CURRENT_LIMIT = true;

  /** Optional “lower” threshold + timer (kept from your originals). */
  public static final int SUPPLY_LOWER_CURRENT_LIMIT = 40;       // SupplyCurrentLowerLimit
  public static final double SUPPLY_CURRENT_LIMIT_TIMEOUT = 1.5; // SupplyCurrentLowerTime (sec)
  public static final boolean USE_SUPPLY_LOWER_CURRENT_LIMIT = true;

  public static final int STATOR_CURRENT_LIMIT = 100;
  public static final boolean USE_STATOR_CURRENT_LIMIT = true;

  /** Phoenix voltage model time constant (sec). */
  public static final double SUPPLY_VOLTAGE_TIME_CONSTANT = 0.02;

  // ---------- Closed-loop gains (defaults) ----------
  // Slot0 = normal operation
  public static final double S0_kP = 2.0;
  public static final double S0_kD = 0.1;
  public static final double S0_kS = 0.225;
  public static final double S0_kG = 0.075;
  public static final double S0_kV = 0.0;
  public static final double S0_kA = 0.0;

  // Slot1 = stow behavior (same as your STOW_GAINS by default)
  public static final double S1_kP = 2.0;
  public static final double S1_kD = 0.1;
  public static final double S1_kS = 0.225;
  public static final double S1_kG = 0.075;
  public static final double S1_kV = 0.0;
  public static final double S1_kA = 0.0;

  // ---------- Tunable mirrors for gains ----------
  public static final LoggedTunableNumber T_S0_kP =
      new LoggedTunableNumber("Elevator/Gains/Slot0/kP", S0_kP);
  public static final LoggedTunableNumber T_S0_kD =
      new LoggedTunableNumber("Elevator/Gains/Slot0/kD", S0_kD);
  public static final LoggedTunableNumber T_S0_kS =
      new LoggedTunableNumber("Elevator/Gains/Slot0/kS", S0_kS);
  public static final LoggedTunableNumber T_S0_kG =
      new LoggedTunableNumber("Elevator/Gains/Slot0/kG", S0_kG);
  public static final LoggedTunableNumber T_S0_kV =
      new LoggedTunableNumber("Elevator/Gains/Slot0/kV", S0_kV);
  public static final LoggedTunableNumber T_S0_kA =
      new LoggedTunableNumber("Elevator/Gains/Slot0/kA", S0_kA);

  public static final LoggedTunableNumber T_S1_kP =
      new LoggedTunableNumber("Elevator/Gains/Slot1/kP", S1_kP);
  public static final LoggedTunableNumber T_S1_kD =
      new LoggedTunableNumber("Elevator/Gains/Slot1/kD", S1_kD);
  public static final LoggedTunableNumber T_S1_kS =
      new LoggedTunableNumber("Elevator/Gains/Slot1/kS", S1_kS);
  public static final LoggedTunableNumber T_S1_kG =
      new LoggedTunableNumber("Elevator/Gains/Slot1/kG", S1_kG);
  public static final LoggedTunableNumber T_S1_kV =
      new LoggedTunableNumber("Elevator/Gains/Slot1/kV", S1_kV);
  public static final LoggedTunableNumber T_S1_kA =
      new LoggedTunableNumber("Elevator/Gains/Slot1/kA", S1_kA);

  // ---------- Motion constraints (meters-based) ----------
  public static final double MAX_ACCEL_MPS2 = 16.0;
  public static final double CRUISE_VEL_MPS = 16.0;
  public static final double GOAL_TOLERANCE_M = Units.inchesToMeters(0.5);

  // ---------- Tunable mirrors for constraints ----------
  public static final LoggedTunableNumber T_MAX_ACCEL_MPS2 =
      new LoggedTunableNumber("Elevator/Constraints/MaxAccelerationMps2", MAX_ACCEL_MPS2);
  public static final LoggedTunableNumber T_CRUISE_VEL_MPS =
      new LoggedTunableNumber("Elevator/Constraints/CruiseVelocityMps", CRUISE_VEL_MPS);
  public static final LoggedTunableNumber T_GOAL_TOLERANCE_M =
      new LoggedTunableNumber("Elevator/Constraints/GoalToleranceM", GOAL_TOLERANCE_M);

  // ---------- Preset positions (meters) ----------
  public static final class Preset {
    public static final double INITIAL_HEIGHT = Units.inchesToMeters(0.0);

    public static final double STOW        = Units.inchesToMeters(-0.1);
    public static final double L1          = Units.inchesToMeters(2.0);
    public static final double L2          = Units.inchesToMeters(6.75);
    public static final double L3          = Units.inchesToMeters(14.5);
    public static final double L4          = Units.inchesToMeters(26.9);
    public static final double ALGAE_HIGH  = Units.inchesToMeters(17.0);
    public static final double ALGAE_LOW   = Units.inchesToMeters(8.5);
  }

  // ---------- Phoenix6 config object (ready-to-apply) ----------
  public static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration();

  static {
    // SIM overrides to match your original SIM tunings.
    switch (Constants.getMode()) {
      case SIM -> {
        T_S0_kP.initDefault(20.0); T_S0_kD.initDefault(0.0); T_S0_kS.initDefault(0.0); T_S0_kG.initDefault(0.0); T_S0_kV.initDefault(0.0); T_S0_kA.initDefault(0.0);
        T_S1_kP.initDefault(20.0); T_S1_kD.initDefault(0.0); T_S1_kS.initDefault(0.0); T_S1_kG.initDefault(0.0); T_S1_kV.initDefault(0.0); T_S1_kA.initDefault(0.0);
        T_MAX_ACCEL_MPS2.initDefault(101.078594);
        T_CRUISE_VEL_MPS.initDefault(11.329982);
        T_GOAL_TOLERANCE_M.initDefault(0.02);
      }
      case REAL, REPLAY -> { /* keep defaults */ }
      default -> {}
    }

    // Output mode
    MOTOR_CONFIG.MotorOutput.Inverted =
        IS_INVERTED ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    MOTOR_CONFIG.MotorOutput.NeutralMode =
        IS_BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // Limits
    MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = USE_SUPPLY_CURRENT_LIMIT;
    MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    MOTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = USE_STATOR_CURRENT_LIMIT;

    if (USE_SUPPLY_LOWER_CURRENT_LIMIT) {
      MOTOR_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = SUPPLY_LOWER_CURRENT_LIMIT;
      MOTOR_CONFIG.CurrentLimits.SupplyCurrentLowerTime = SUPPLY_CURRENT_LIMIT_TIMEOUT;
    } else {
      MOTOR_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 0.0;
      MOTOR_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 0.0;
    }

    // Unit scaling: rotor -> mechanism (sprocket rotations).
    MOTOR_CONFIG.Feedback.SensorToMechanismRatio = GEARING;

    MOTOR_CONFIG.Voltage.SupplyVoltageTimeConstant = SUPPLY_VOLTAGE_TIME_CONSTANT;

    // Soft limits (convert meters -> *mechanism* rotations; i.e., meters / circumference)
    final double fwdSoftLimitRot = MAX_HEIGHT_METERS / SPROCKET_CIRCUMFERENCE_METERS;
    final double revSoftLimitRot = MIN_HEIGHT_METERS / SPROCKET_CIRCUMFERENCE_METERS;
    MOTOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = fwdSoftLimitRot;
    MOTOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    MOTOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = revSoftLimitRot;
    MOTOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    // Motion Magic (meters/sec -> mech rotations/sec)
    final double cruiseRps = T_CRUISE_VEL_MPS.get() / SPROCKET_CIRCUMFERENCE_METERS;
    final double accelRps2 = T_MAX_ACCEL_MPS2.get() / SPROCKET_CIRCUMFERENCE_METERS;
    MOTOR_CONFIG.MotionMagic.MotionMagicCruiseVelocity = cruiseRps;
    MOTOR_CONFIG.MotionMagic.MotionMagicAcceleration = accelRps2;

    // Closed-loop gains
    MOTOR_CONFIG.Slot0.kP = T_S0_kP.get(); MOTOR_CONFIG.Slot0.kD = T_S0_kD.get();
    MOTOR_CONFIG.Slot0.kS = T_S0_kS.get(); MOTOR_CONFIG.Slot0.kG = T_S0_kG.get();
    MOTOR_CONFIG.Slot0.kV = T_S0_kV.get(); MOTOR_CONFIG.Slot0.kA = T_S0_kA.get();

    MOTOR_CONFIG.Slot1.kP = T_S1_kP.get(); MOTOR_CONFIG.Slot1.kD = T_S1_kD.get();
    MOTOR_CONFIG.Slot1.kS = T_S1_kS.get(); MOTOR_CONFIG.Slot1.kG = T_S1_kG.get();
    MOTOR_CONFIG.Slot1.kV = T_S1_kV.get(); MOTOR_CONFIG.Slot1.kA = T_S1_kA.get();
  }
}
