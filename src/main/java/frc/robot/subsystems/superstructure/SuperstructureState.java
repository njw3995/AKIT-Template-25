package frc.robot.subsystems.superstructure;

/**
 * Pure type definitions for the Superstructure coordinator.
 *
 * <p>Keep this file device-agnostic: no subsystem imports, no logic. It defines the public
 * vocabulary (states/modes) the rest of the superstructure uses.
 */
public final class SuperstructureState {
  private SuperstructureState() {}

  /** High-level game mode for mechanism behavior. */
  public enum Mode {
    /** Coral handling behavior/presets. */
    CORAL,
    /** Algae handling behavior/presets. */
    ALGAE
  }

  /** Operator automation policy for scoring. */
  public enum AutomationLevel {
    /** Nothing happens automatically (manual eject only). */
    MANUAL,
    /** When aligned & at pose, auto-eject the coral. */
    AUTO_RELEASE,
    /** Reserved for a future mode (auto drive + manual release). */
    AUTO_DRIVE_AND_MANUAL_RELEASE
  }

  /** Operator- or routine-requested state (high-level intent). */
  public enum WantedState {
    DEFAULT,
    STOP,
    STOW,
    HOME,
    INTAKE_CORAL,
    HOLD_CORAL,
    SCORE_L1,
    SCORE_L2,
    SCORE_L3,
    SCORE_L4,
    INTAKE_ALGAE_GROUND,
    INTAKE_ALGAE_REEF,
    HOLD_ALGAE,
    SCORE_ALGAE_PROCESSOR
  }

  /** Internally applied state after transition logic. */
  public enum CurrentState {
    STOPPED,
    HOMING,
    STOWING,
    HOLDING_CORAL,
    HOLDING_ALGAE,
    INTAKING_CORAL,
    SCORING_L1,
    SCORING_L2,
    SCORING_L3,
    SCORING_L4,
    INTAKING_ALGAE_GROUND,
    INTAKING_ALGAE_REEF,
    SCORING_ALGAE_PROCESSOR
  }
}
