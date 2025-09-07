package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  RobotState state;
  EndEffectorIO io;

  protected final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;
  private final Alert tempFault;

  private final Debouncer algaeDebouncer = new Debouncer(1, Debouncer.DebounceType.kRising);
  private final LinearFilter currentFilter = LinearFilter.movingAverage(10);
  private final LinearFilter velocityFilter = LinearFilter.movingAverage(10);

  public EndEffector(EndEffectorIO io, RobotState state) {
    this.io = io;
    this.state = state;

    disconnected = new Alert(getName() + " motor disconnected!", Alert.AlertType.kWarning);
    tempFault = new Alert(getName() + " motor too hot! 🥵", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);

    disconnected.set(!motorConnectedDebouncer.calculate(getConnected()) && !Robot.isJITing());
    tempFault.set(inputs.data.tempFault());

    LoggedTracer.record("EndEffector");
  }

  public boolean hasCoral() {
    return getIntakeBeamBreakDetected() || getPlacementBeamBreakDetected();
  }

  public boolean hasAlgae() {
    if (Constants.getMode() != Constants.Mode.SIM) {
      double filteredCurrent = currentFilter.calculate(getTorqueCurrentAmps());
      double filteredVelocity = velocityFilter.calculate(getVelocityRadsPerSec());
      boolean hasAlgaeInputSignal =
          filteredCurrent > EndEffectorConstants.HAS_ALGAE_CURRENT
              && filteredVelocity < EndEffectorConstants.HAS_ALGAE_MAX_VELOCITY;
      // Take raw input signal that has some noise and run it through a 1 second debouncer
      boolean hasAlgaeOutputSignal = algaeDebouncer.calculate(hasAlgaeInputSignal);
      return hasAlgaeOutputSignal;
    } else {
      return true; // Its sim lol
    }
  }

  public boolean coralArmed() {
    return getIntakeBeamBreakDetected() && getPlacementBeamBreakDetected();
  }

  /** stops running the intake motor Do not use this to hold algae. Use {@code holdAlgae()}. */
  public Command stopMotorCommand() {
    return new InstantCommand(() -> io.stop());
  }

  /**
   * Runs the intake until the robot has coral, and then slowly secures it to a consistent place
   *
   * @return the command
   */
  public Command intakeCoralCommand() { // TODO: Idk man this needs constants ig?
    return new SequentialCommandGroup(
        new InstantCommand(() -> io.runVolts(2)),
        new WaitUntilCommand(this::hasCoral),
        new InstantCommand(() -> io.runVolts(2)),
        new WaitUntilCommand(this::coralArmed),
        new InstantCommand(() -> io.runVolts(1)),
        new WaitUntilCommand(this::hasCoral),
        stopMotorCommand());
  }

  public Command intakeAlgaeCommand() {
    return new InstantCommand(() -> io.runVolts(2)).andThen(new RunCommand(() -> {}));
  }

  /**
   * Runs the motor until there isn't coral in the system
   *
   * @return the command
   */
  public Command depositCoralCommand() {
    return new InstantCommand(() -> io.runVolts(2))
        .andThen(new RunCommand(() -> {}))
        .finallyDo(this::stopMotorCommand);
  }

  /**
   * Runs the motor until there isn't coral in the system
   *
   * @return the command
   */
  public Command depositL1CoralCommand() {
    return new InstantCommand(() -> io.runVolts(2))
        .andThen(new RunCommand(() -> {}))
        .finallyDo(this::stopMotorCommand);
  }

  public Command depositAlgaeCommand() {
    return new InstantCommand(() -> io.runVolts(2))
        .andThen(new RunCommand(() -> {}))
        .until(() -> (!hasAlgae()))
        .finallyDo(this::stopMotorCommand);
  }

  public Command waitForCoralIntakeCommand() {
    return new WaitCommand(Constants.AutoConstants.INTAKE_CORAL_WAIT_TIME);
  }

  public Command waitForAlgaeIntakeCommand() {
    return new WaitCommand(Constants.AutoConstants.INTAKE_ALGAE_WAIT_TIME);
  }

  public Command waitForAlgaeScoreCommand() {
    return new WaitCommand(Constants.AutoConstants.SCORE_ALGAE_WAIT_TIME);
  }

  public Command waitForCoralScoreCommand() {
    return new WaitCommand(Constants.AutoConstants.SCORE_CORAL_WAIT_TIME);
  }

  // IOData Getters
  public double getPositionRads() {
    return inputs.data.positionRads();
  }

  public double getVelocityRadsPerSec() {
    return inputs.data.velocityRadsPerSec();
  }

  public double getAppliedVoltage() {
    return inputs.data.appliedVoltage();
  }

  public double getSupplyCurrentAmps() {
    return inputs.data.supplyCurrentAmps();
  }

  public double getTorqueCurrentAmps() {
    return inputs.data.torqueCurrentAmps();
  }

  public double getTempCelsius() {
    return inputs.data.tempCelsius();
  }

  public boolean getTempFault() {
    return inputs.data.tempFault();
  }

  public boolean getIntakeBeamBreakDetected() {
    return inputs.data.intakeBeamBreakDetected();
  }

  public boolean getPlacementBeamBreakDetected() {
    return inputs.data.placementBeamBreakDetected();
  }

  public boolean getConnected() {
    return inputs.data.connected();
  }
}
