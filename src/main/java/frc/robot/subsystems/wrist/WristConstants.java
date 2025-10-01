package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

/** All values live here; IO just applies MOTOR_CONFIG and uses the doubles. */
public final class WristConstants {
  private WristConstants() {}

  // ---------- Mechanism / units ----------
  /** Motor-to-joint ratio (sensor-to-mechanism ratio in Phoenix). */
  public static final double GEARING = 90.0;
  /** Joint soft limits (mechanism space). */
  public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-77.0);

  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(75.0);
  /** Link length for inertia/CG modeling (meters). */
  public static final double LENGTH_METERS = 0.5;
  /** DCMotor model used by sims/feeds. */
  public static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX60Foc(1);

  // ---------- Electrical limits / mode ----------
  public static final boolean IS_BRAKE = true;
  public static final boolean IS_INVERTED = false; // true -> CCW Positive, false -> CW Positive
  public static final int SUPPLY_CURRENT_LIMIT = 80;
  public static final boolean USE_SUPPLY_CURRENT_LIMIT = true;
  public static final int STATOR_CURRENT_LIMIT = 100;
  public static final boolean USE_STATOR_CURRENT_LIMIT = true;
  /** Phoenix voltage model time constant (sec). */
  public static final double SUPPLY_VOLTAGE_TIME_CONSTANT = 0.02;

  // ---------- Closed-loop gains (defaults) ----------
  /** Slot0 = lighter load (no algae). */
  public static final double S0_kP = 125.0;

  public static final double S0_kD = 0.0;
  public static final double S0_kS = 0.24274;
  public static final double S0_kV = 0.0;
  public static final double S0_kA = 0.0;
  /** Optional gravity FF for sim/real if you use it. */
  public static final double S0_kG = 0.66177;

  /** Slot1 = heavier load (with algae). */
  public static final double S1_kP = 125.0;

  public static final double S1_kD = 0.0;
  public static final double S1_kS = 0.65347;
  public static final double S1_kV = 0.0;
  public static final double S1_kA = 0.0;
  /** Optional gravity FF for heavier load. */
  public static final double S1_kG = 2.0762;

  // ---------- Tunable mirrors for gains (initialized to defaults above) ----------
  public static final LoggedTunableNumber T_S0_kP =
      new LoggedTunableNumber("Wrist/Gains/Slot0/kP", S0_kP);
  public static final LoggedTunableNumber T_S0_kD =
      new LoggedTunableNumber("Wrist/Gains/Slot0/kD", S0_kD);
  public static final LoggedTunableNumber T_S0_kS =
      new LoggedTunableNumber("Wrist/Gains/Slot0/kS", S0_kS);
  public static final LoggedTunableNumber T_S0_kV =
      new LoggedTunableNumber("Wrist/Gains/Slot0/kV", S0_kV);
  public static final LoggedTunableNumber T_S0_kA =
      new LoggedTunableNumber("Wrist/Gains/Slot0/kA", S0_kA);

  public static final LoggedTunableNumber T_S1_kP =
      new LoggedTunableNumber("Wrist/Gains/Slot1/kP", S1_kP);
  public static final LoggedTunableNumber T_S1_kD =
      new LoggedTunableNumber("Wrist/Gains/Slot1/kD", S1_kD);
  public static final LoggedTunableNumber T_S1_kS =
      new LoggedTunableNumber("Wrist/Gains/Slot1/kS", S1_kS);
  public static final LoggedTunableNumber T_S1_kV =
      new LoggedTunableNumber("Wrist/Gains/Slot1/kV", S1_kV);
  public static final LoggedTunableNumber T_S1_kA =
      new LoggedTunableNumber("Wrist/Gains/Slot1/kA", S1_kA);

  // ---------- Motion constraints (defaults) ----------
  /** Motion Magic acceleration (rotations/s^2). */
  public static final double MAX_ACCEL_RPS2 = 2.0;
  /** Motion Magic cruise velocity (rotations/s). */
  public static final double CRUISE_VEL_RPS = 5.0;
  /** Position at-goal tolerance (radians). */
  public static final double GOAL_TOLERANCE_RAD = Units.degreesToRadians(1.5);

  // ---------- Tunable mirrors for constraints (initialized to defaults above) ----------
  public static final LoggedTunableNumber T_MAX_ACCEL_RPS2 =
      new LoggedTunableNumber("Wrist/Constraints/MaxAccelerationRps2", MAX_ACCEL_RPS2);
  public static final LoggedTunableNumber T_CRUISE_VEL_RPS =
      new LoggedTunableNumber("Wrist/Constraints/CruiseVelocityRps", CRUISE_VEL_RPS);
  public static final LoggedTunableNumber T_GOAL_TOLERANCE_RAD =
      new LoggedTunableNumber("Wrist/Constraints/GoalToleranceRad", GOAL_TOLERANCE_RAD);

  // ---------- Preset positions (angles you command to) ----------
  public static final class Preset {
    public static final Rotation2d STOW = Rotation2d.fromDegrees(6.25);
    public static final Rotation2d AUTO_HOVER_L4 = Rotation2d.fromDegrees(24.15);
    public static final Rotation2d HOVER_L4 = Rotation2d.fromDegrees(29.15);
    public static final Rotation2d HOVER_L3 = Rotation2d.fromDegrees(31.15);
    public static final Rotation2d HOVER_L2 = Rotation2d.fromDegrees(29.15);
    public static final Rotation2d HOVER_L1 = Rotation2d.fromDegrees(14.15);
    public static final Rotation2d DUNK_L4 = Rotation2d.fromDegrees(39.15);
    public static final Rotation2d DUNK_L3 = Rotation2d.fromDegrees(39.15);
    public static final Rotation2d DUNK_L2 = Rotation2d.fromDegrees(39.15);
    public static final Rotation2d DUNK_L1 = Rotation2d.fromDegrees(23.15);
    public static final Rotation2d GROUND_ALGAE = Rotation2d.fromDegrees(69.15);
    public static final Rotation2d REEF_ALGAE_LOW = Rotation2d.fromDegrees(40.15);
    public static final Rotation2d REEF_ALGAE_HIGH = Rotation2d.fromDegrees(36.15);
    public static final Rotation2d PROCESSOR = Rotation2d.fromDegrees(34.15);
    public static final Rotation2d L1 = Rotation2d.fromDegrees(6.25);
  }

  // ---------- Phoenix6 config object (ready-to-apply) ----------
  public static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration();

  static {
    // If you want mode-specific tweaks, branch here (kept for parity with EndEffector/Elevator).
    switch (Constants.getMode()) {
      case REAL, REPLAY -> {
        // leave the student-facing numbers as-is (tunables already seeded from defaults)
      }
      case SIM -> {
        // Optionally tone down limits in sim; we keep your defaults, but you could:
        // T_S0_kP.set(20.0); T_S1_kP.set(20.0); etc.
      }
    }

    // Build the Phoenix config once from the tunables (no logic in IO).
    MOTOR_CONFIG.MotorOutput.Inverted =
        IS_INVERTED ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    MOTOR_CONFIG.MotorOutput.NeutralMode =
        IS_BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = USE_SUPPLY_CURRENT_LIMIT;
    MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    MOTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = USE_STATOR_CURRENT_LIMIT;

    MOTOR_CONFIG.Feedback.SensorToMechanismRatio = GEARING;

    MOTOR_CONFIG.Voltage.SupplyVoltageTimeConstant = SUPPLY_VOLTAGE_TIME_CONSTANT;

    // Joint soft limits
    MOTOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.getRotations();
    MOTOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    MOTOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.getRotations();
    MOTOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    // Motion Magic constraints (from tunables)
    MOTOR_CONFIG.MotionMagic.MotionMagicAcceleration = T_MAX_ACCEL_RPS2.get();
    MOTOR_CONFIG.MotionMagic.MotionMagicCruiseVelocity = T_CRUISE_VEL_RPS.get();

    // Closed-loop gains (from tunables)
    MOTOR_CONFIG.Slot0.kP = T_S0_kP.get();
    MOTOR_CONFIG.Slot0.kD = T_S0_kD.get();
    MOTOR_CONFIG.Slot0.kS = T_S0_kS.get();
    MOTOR_CONFIG.Slot0.kV = T_S0_kV.get();
    MOTOR_CONFIG.Slot0.kA = T_S0_kA.get();

    MOTOR_CONFIG.Slot1.kP = T_S1_kP.get();
    MOTOR_CONFIG.Slot1.kD = T_S1_kD.get();
    MOTOR_CONFIG.Slot1.kS = T_S1_kS.get();
    MOTOR_CONFIG.Slot1.kV = T_S1_kV.get();
    MOTOR_CONFIG.Slot1.kA = T_S1_kA.get();
  }
}
