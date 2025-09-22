package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

/** All values live here; IO just applies MOTOR_CONFIG and uses the doubles. */
public final class EndEffectorConstants {
  private EndEffectorConstants() {}

  // ---------- Mechanism / units ----------
  /** Motor-to-roller ratio (sensor-to-mechanism ratio in Phoenix). */
  public static final double GEARING = 5.0;

  // ---------- Electrical limits / mode ----------
  public static final boolean IS_BRAKE = false;
  public static final boolean IS_INVERTED = true; // true -> CCW Positive, false -> CW Positive
  public static final int SUPPLY_CURRENT_LIMIT = 40;
  public static final boolean USE_SUPPLY_CURRENT_LIMIT = true;
  public static final int STATOR_CURRENT_LIMIT = 80;
  public static final boolean USE_STATOR_CURRENT_LIMIT = true;
  /** Phoenix voltage model time constant (sec). */
  public static final double SUPPLY_VOLTAGE_TIME_CONSTANT = 0.02;

  // ---------- Detection / filtering ----------
  /** Algae detection threshold: current must be >= and |velocity| must be <= these. */
  public static final double HAS_ALGAE_CURRENT_AMPS = 10.0;

  public static final double HAS_ALGAE_MAX_VELOCITY_RAD_PER_SEC = 50.0;
  /** Debounce seconds for algae detection (periodic helper). */
  public static final double HAS_ALGAE_DEBOUNCE_SEC = 1.0;
  /** Moving-average samples for current/velocity filters in periodic helper. */
  public static final int CURRENT_FILTER_SAMPLE_SIZE = 10;

  public static final int VELOCITY_FILTER_SAMPLE_SIZE = 10;

  // ---------- Voltage tables (fill these during tuning) ----------
  public static final class CollectingVoltages {
    public static final double ALGAE = 0.0;
    public static final double CORAL = 2.5;
  }

  public static final class HoldingVoltages {
    public static final double ALGAE = 0.0;
    public static final double ALGAE_HARDER = 0.0;
    public static final double CORAL = 0.0;
  }

  public static final class EjectingVoltages {
    public static final double ALGAE = 0.0;
    public static final double ALGAE_PROCESSOR = 0.0;
    public static final double CORAL_L1 = 0.0;
    public static final double CORAL = 0.0;
  }

  public static final class IndexingVoltages {
    public static final double CORAL_FORWARD = 1.5;
    public static final double CORAL_BACKWARD = -1.5;
  }

  // ---------- Phoenix6 config object (ready-to-apply) ----------
  public static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration();

  static {
    // If you want mode-specific tweaks, branch here.
    switch (Constants.getMode()) {
      case REAL, REPLAY -> {
        // leave the student-facing numbers as-is
      }
      case SIM -> {
        // you can tone down limits for sim if desired
      }
    }

    // Build the Phoenix config once from the doubles above (no logic in IO).
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
  }
}
