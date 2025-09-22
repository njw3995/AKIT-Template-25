package frc.robot.subsystems.superstructure;

/**
 * Pure type definitions for the Superstructure coordinator.
 * <p>
 * Keep this file device-agnostic: no subsystem imports, no logic. It defines
 * the public vocabulary (states/modes) the rest of the superstructure uses.
 */
public final class SuperstructureState {
  private SuperstructureState() {}
  public enum Mode { CORAL, ALGAE }

  /** Operator- or routine-requested state (high level intent). */
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
    NO_PIECE,
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
