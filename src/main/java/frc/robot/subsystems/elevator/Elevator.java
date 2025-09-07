package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import frc.robot.util.LoggedTracer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;

  private ElevatorPositions position;
  private boolean isClosedLoop;

  private final Alert motorDisconnectedAlert =
      new Alert("Elevator leader motor disconnected!", Alert.AlertType.kWarning);
  private final Alert followerDisconnectedAlert =
      new Alert("Elevator follower motor disconnected!", Alert.AlertType.kWarning);
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer followerMotorConnectedDebouncer =
      new Debouncer(0.5, DebounceType.kFalling);

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.inputs = new ElevatorIOInputsAutoLogged();

    position = ElevatorPositions.STOW;
    isClosedLoop = true;
  }

  private void periodic() {
    io.updateInputs(inputs);

    motorDisconnectedAlert.set(
        !motorConnectedDebouncer.calculate(inputs.data.motorConnected()) && !Robot.isJITing());
    followerDisconnectedAlert.set(
        !followerMotorConnectedDebouncer.calculate(inputs.data.followerConnected())
            && !Robot.isJITing());

    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("Elevator/Position", position.name());

    Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", position.getPosition());

    RobotState.getInstance()
        .setElevatorExtensionPercent(
            getPositionMeters(inputs) / ElevatorConstants.ELEVATOR_PARAMETERS.MAX_HEIGHT_METERS());

    LoggedTracer.record("Elevator");
  }

  /**
   * Gets the elevator position enum for a given reef state.
   *
   * @param newPosition The reef state.
   * @return The corresponding elevator position.
   */
  private ElevatorPositions getPosition(ElevatorPositions newPosition) {
    // return ElevatorConstants.REEF_STATE_ELEVATOR_POSITION_MAP.get(newPosition);
    return ElevatorPositions.ALGAE_INTAKE;
  }

  /**
   * Gets the current position of the elevator.
   *
   * @return The current elevator position.
   */
  private double getPositionMeters(ElevatorIOInputs inputs) {
    return inputs.data.positionMeters();
  }

  /**
   * Gets the feedforward characterization velocity.
   *
   * @return The feedforward characterization velocity.
   */
  private double getFFCharacterizationVelocity(ElevatorIOInputs inputs) {
    return inputs.data.velocityMetersPerSec();
  }

  /**
   * Sets the control gains for the elevator.
   *
   * @param kP The proportional gain.
   * @param kD The derivative gain.
   * @param kS The static gain.
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   * @param kG The gravity gain.
   */
  private void setGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    io.updateGains(kP, kD, kS, kV, kA, kG);
  }

  /**
   * Sets the motion constraints for the elevator.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param cruisingVelocity The cruising velocity.
   */
  private void setConstraints(double maxAcceleration, double cruisingVelocity) {
    io.updateConstraints(maxAcceleration, cruisingVelocity);
  }

  /**
   * Checks if the elevator is at the goal position within a specified tolerance.
   *
   * @param position The target position in meters.
   * @return true if the current position is within the goal tolerance of the target position, false
   *     otherwise.
   */
  private boolean atGoal(double position) {
    // return Math.abs(position - inputs.positionMeters)
    //     <= ElevatorConstants.CONSTRAINTS.goalToleranceMeters().get();
    return false;
  }

  /**
   * Checks if the elevator is at the goal position.
   *
   * @return True if the elevator is at the goal position, false otherwise.
   */
  @AutoLogOutput(key = "Elevator/At Goal")
  private boolean atGoal() {
    // return Math.abs(inputs.positionGoalMeters - inputs.positionMeters)
    //     <= ElevatorConstants.CONSTRAINTS.goalToleranceMeters().get();
    return false;
  }

  /**
   * Waits until the elevator is at the goal position.
   *
   * @return A command that waits until the elevator is at the goal position.
   */
  private Command waitUntilAtGoal() {
    return Commands.waitSeconds(0.02).andThen(Commands.waitUntil(() -> atGoal()));
  }

  /**
   * Checks if the elevator is within a fast scoring tolerance of the goal position.
   *
   * @return A BooleanSupplier that returns true if the elevator is within the fast scoring
   *     tolerance, false otherwise.
   */
  private BooleanSupplier inFastScoringTolerance() {
    // return () -> Math.abs(inputs.positionMeters - inputs.positionGoalMeters) <= 0.03;
    return () -> false;
  }

  /**
   * Runs the SysId routine for the elevator subsystem.
   *
   * @param subsystem The subsystem to run the SysId routine on.
   * @return A command that runs the SysId routine.
   */
  private Command sysIdRoutine(Subsystem subsystem) {

    SysIdRoutine characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(6),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Elevator/SysID State", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, subsystem));

    return Commands.sequence(
        characterizationRoutine
            .quasistatic(Direction.kForward)
            .until(
                () ->
                    Elevator.this.atGoal(
                        ElevatorPositions.L4.getPosition() - Units.inchesToMeters(12.0))),
        characterizationRoutine
            .quasistatic(Direction.kReverse)
            .until(
                () ->
                    Elevator.this.atGoal(
                        ElevatorPositions.STOW.getPosition() + Units.inchesToMeters(12.0))),
        characterizationRoutine
            .dynamic(Direction.kForward)
            .until(
                () ->
                    Elevator.this.atGoal(
                        ElevatorPositions.L4.getPosition() - Units.inchesToMeters(12.0))),
        characterizationRoutine
            .dynamic(Direction.kReverse)
            .until(
                () ->
                    Elevator.this.atGoal(
                        ElevatorPositions.STOW.getPosition() + Units.inchesToMeters(12.0))),
        subsystem.runOnce(() -> setPosition()));

    // subsystem.runOnce(() -> setPosition(() -> ReefState.STOW)));
  }

  /**
   * Sets the position of the elevator.
   *
   * @return A command that sets the elevator position.
   */
  private void setPosition() {
    // setPosition(() -> RobotState.getOIData().currentReefHeight());
  }

  /**
   * Sets the position of the elevator.
   *
   * @param positionRadians The desired elevator position.
   * @return A command that sets the elevator position.
   */
  private void setPosition(Supplier<ElevatorPositions> newPosition) {
    isClosedLoop = true;
    Elevator.this.position = Elevator.this.getPosition(newPosition.get());
    io.setPositionGoal(Elevator.this.position.getPosition());
  }

  public class ElevatorCSB extends SubsystemBase {

    @Override
    public void periodic() {
      Elevator.this.periodic();
    }

    public Command setPosition() {
      return this.runOnce(() -> Elevator.this.setPosition());
    }

    public ElevatorPositions getPosition() {
      return Elevator.this.position;
    }

    public Command setPosition(Supplier<ElevatorPositions> newPosition) {
      return this.runOnce(() -> Elevator.this.setPosition(newPosition));
    }

    public Command setVoltage(double volts) {
      return this.runEnd(
          () -> {
            isClosedLoop = false;
            io.setVoltage(volts);
          },
          () -> io.setVoltage(0.0));
    }

    public Command resetPosition() {
      return runOnce(() -> Elevator.this.position = ElevatorPositions.STOW)
          .andThen(
              runOnce(
                  () -> io.setPosition(ElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS())));
    }

    public Command sysIdRoutine() {
      return Elevator.this.sysIdRoutine(this);
    }

    public double getPositionMeters() {
      return Elevator.this.getPositionMeters(inputs);
    }

    public double getFFCharacterizationVelocity() {
      return Elevator.this.getFFCharacterizationVelocity(inputs);
    }

    public void setGains(double kP, double kD, double kS, double kV, double kA, double kG) {
      Elevator.this.setGains(kP, kD, kS, kV, kA, kG);
    }

    public void setConstraints(double maxAcceleration, double cruisingVelocity) {
      Elevator.this.setConstraints(maxAcceleration, cruisingVelocity);
    }

    public boolean atGoal() {
      return Elevator.this.atGoal();
    }

    public Command waitUntilAtGoal() {
      return Elevator.this.waitUntilAtGoal();
    }

    public BooleanSupplier inFastScoringTolerance() {
      return Elevator.this.inFastScoringTolerance();
    }
  }

  public class ElevatorFSM {

    public void periodic() {
      Elevator.this.periodic();
    }

    public ElevatorPositions getPosition() {
      return Elevator.this.position;
    }

    public void setPosition() {
      Elevator.this.setPosition();
    }

    public void setPosition(Supplier<ElevatorPositions> newPosition) {
      Elevator.this.setPosition(newPosition);
    }

    public Command resetPosition() {
      return Commands.runOnce(() -> Elevator.this.position = ElevatorPositions.STOW)
          .andThen(
              Commands.runOnce(
                  () -> io.setPosition(ElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS())));
    }

    public double getPositionMeters() {
      return Elevator.this.getPositionMeters(inputs);
    }

    public double getFFCharacterizationVelocity() {
      return Elevator.this.getFFCharacterizationVelocity(inputs);
    }

    public void setGains(double kP, double kD, double kS, double kV, double kA, double kG) {
      Elevator.this.setGains(kP, kD, kS, kV, kA, kG);
    }

    public void setConstraints(double maxAcceleration, double cruisingVelocity) {
      Elevator.this.setConstraints(maxAcceleration, cruisingVelocity);
    }

    public boolean atGoal() {
      return Elevator.this.atGoal();
    }

    public Command waitUntilAtGoal() {
      return Elevator.this.waitUntilAtGoal();
    }

    public BooleanSupplier inFastScoringTolerance() {
      return Elevator.this.inFastScoringTolerance();
    }
  }

  public ElevatorFSM getFSM() {
    return new ElevatorFSM();
  }

  public ElevatorCSB getCSB() {
    return new ElevatorCSB();
  }
}
