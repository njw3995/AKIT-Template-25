package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.wrist.WristConstants.WristState;
import frc.robot.util.LoggedTracer;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs;
  private boolean isClosedLoop;

  private final Alert motorDisconnectedAlert =
      new Alert("Wrist motor disconnected!", Alert.AlertType.kWarning);
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  @Getter
  @AutoLogOutput(key = "Wrist/Wrist Goal")
  private WristState wristGoal;

  public Wrist(WristIO io) {
    this.io = io;
    inputs = new WristIOInputsAutoLogged();
    isClosedLoop = true;

    wristGoal = WristState.STOW_DOWN;
  }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Wrist", inputs);

    motorDisconnectedAlert.set(
        !motorConnectedDebouncer.calculate(inputs.data.motorConnected()) && !Robot.isJITing());

    if (isClosedLoop) io.setWristPosition(wristGoal.getAngle());

    // if (RobotState.isHasAlgae()
    //     && Set.of(
    //             ManipulatorRollerState.STOP,
    //             ManipulatorRollerState.ALGAE_INTAKE,
    //             ManipulatorRollerState.CORAL_INTAKE)
    //         .contains(rollerGoal)) {
    //   io.setRollerVoltage(holdVoltage());
    // } else {
    //   io.setRollerVoltage(rollerGoal.getVoltage());
    // }

    // if (hasAlgae() && RobotState.isIntakingAlgae()) {
    //   RobotState.setHasAlgae(true);
    // }
    LoggedTracer.record("Wrist");
  }

  public Rotation2d getWristAngle() {
    return Rotation2d.fromRadians(inputs.data.positionRad());
  }

  // public Command sysIdRoutine(V2_RedundancySuperstructure superstructure) {
  //   SysIdRoutine algaeCharacterizationRoutine =
  //       new SysIdRoutine(
  //           new SysIdRoutine.Config(
  //               Volts.of(0.2).per(Second),
  //               Volts.of(3.5),
  //               Seconds.of(5),
  //               (state) -> Logger.recordOutput("Manipulator/SysID State", state.toString())),
  //           new SysIdRoutine.Mechanism(
  //               (volts) -> io.setArmVoltage(volts.in(Volts)), null, superstructure));
  //   return Commands.sequence(
  //       superstructure.runGoal(V2_RedundancySuperstructureStates.OVERRIDE),
  //       Commands.runOnce(() -> isClosedLoop = false),
  //       algaeCharacterizationRoutine.quasistatic(Direction.kForward),
  //       Commands.waitSeconds(.25),
  //       algaeCharacterizationRoutine.quasistatic(Direction.kReverse),
  //       Commands.waitSeconds(.25),
  //       algaeCharacterizationRoutine.dynamic(Direction.kForward),
  //       Commands.waitSeconds(.25),
  //       algaeCharacterizationRoutine.dynamic(Direction.kReverse));
  // }

  public void setAlgaeArmGoal(WristState goal) {
    isClosedLoop = true;
    wristGoal = goal;
  }

  public void updateArmGains(
      double kP0,
      double kD0,
      double kS0,
      double kV0,
      double kA0,
      double kG0,
      double kP1,
      double kD1,
      double kS1,
      double kV1,
      double kA1,
      double kG1) {
    io.updateSlot0WristGains(kP0, kD0, kS0, kV0, kA0, kG0);
    io.updateSlot1WristGains(kP1, kD1, kS1, kV1, kA1, kG1);
  }

  public void updateArmConstraints(double maxAcceleration, double maxVelocity) {
    io.updateWristConstraints(maxAcceleration, maxVelocity);
  }

  @AutoLogOutput(key = "Wrist/Wrist At Goal")
  public boolean algaeArmAtGoal() {
    return inputs.data.positionRad() - wristGoal.getAngle().getRadians()
        <= WristConstants.CONSTRAINTS.GOAL_TOLERANCE_RADIANS().get();
  }

  public Command waitUntilAlgaeArmAtGoal() {
    return Commands.sequence(Commands.waitSeconds(0.02), Commands.waitUntil(this::algaeArmAtGoal));
  }

  public Command homingSequence() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              isClosedLoop = false;
              io.wristMax();
            }),
        Commands.runEnd(() -> io.setWristVoltage(-6), () -> io.setWristVoltage(0))
            .until(() -> Math.abs(inputs.data.torqueCurrentAmps()) > 40),
        Commands.runOnce(() -> io.zeroWristPosition()));
  }
}
