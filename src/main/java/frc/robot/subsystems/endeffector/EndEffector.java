package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
