// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorIO;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.endeffector.EndEffectorIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructurePose;
import frc.robot.subsystems.vision.CameraConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOTalonFX;
import frc.robot.util.OperatorDashboard;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final Wrist wrist;
  private final EndEffector endEffector;
  private final OperatorDashboard operatorDashboard;

  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL -> {
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(DriveConstants.moduleConfigsComp[0]),
                new ModuleIOTalonFX(DriveConstants.moduleConfigsComp[1]),
                new ModuleIOTalonFX(DriveConstants.moduleConfigsComp[2]),
                new ModuleIOTalonFX(DriveConstants.moduleConfigsComp[3]));
        elevator = new Elevator(new ElevatorIOTalonFX());
        wrist = new Wrist(new WristIOTalonFX());
        endEffector = new EndEffector(new EndEffectorIOTalonFX());
        vision = new Vision(CameraConstants.RobotCameras.CAMERAS);
      }

      case SIM -> {
        drive =
            new Drive(
                new GyroIO() {}, // simple stub
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        wrist = new Wrist(new WristIOSim());
        endEffector = new EndEffector(new EndEffectorIOSim());
        vision = new Vision();
      }

      default /* REPLAY */ -> {
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        wrist = new Wrist(new WristIO() {});
        endEffector = new EndEffector(new EndEffectorIO() {});
        vision = new Vision();
      }
    }

    operatorDashboard = new OperatorDashboard();

    // Superstructure owns the high-level sequencing
    superstructure = new Superstructure(drive, elevator, wrist, endEffector, operatorDashboard);
    vision.setYawSupplier(drive::getGyroRotation);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> true));

    Trigger toggleStateButton = driver.leftBumper();
    Trigger intakeButton = driver.leftTrigger();
    Trigger scoreConfirmButton = driver.rightBumper();
    Trigger scoreIntentButton = driver.rightTrigger();
    Trigger wristToggle = driver.a();
    Trigger autoAlignLeft = driver.povLeft().or(driver.povDownLeft()).or(driver.povDownRight());
    // Trigger autoAlignAuto = driver.y();
    Trigger autoAlignRight = driver.povRight().or(driver.povUpLeft()).or(driver.povUpRight());
    Trigger zero = driver.back();
    Trigger coralUnstuck = driver.b();

    Trigger elevatorTargetL1 = new Trigger(operator.a());
    Trigger elevatorTargetL2 = new Trigger(operator.x());
    Trigger elevatorTargetL3 = new Trigger(operator.y());
    Trigger elevatorTargetL4 = new Trigger(operator.b());
    Trigger elevatorDeploy = new Trigger(operator.rightTrigger());
    Trigger elevatorStow = new Trigger(operator.leftTrigger());
    Trigger climbToggle = new Trigger(operator.rightBumper());
    Trigger climbStow = new Trigger(operator.leftBumper());
    Trigger funnelDrop = new Trigger(operator.back());
    Trigger resetFunnel = new Trigger(driver.start());

    // resetFunnel.onTrue(leds.indicateDropCoral());
    // zero.onTrue(drive.runOnce(() -> drive.resetRotation(drive.getOperatorForwardDirection())));

    toggleStateButton.onTrue(superstructure.toggleModeCommand());
    wristToggle.onTrue(superstructure.wristToggleCommand());

    intakeButton
        .whileTrue(superstructure.intakeStartCommand())
        .onFalse(superstructure.intakeStopCommand());

    scoreConfirmButton
        .whileTrue(superstructure.scoreStartCommand())
        .onFalse(superstructure.scoreStopCommand());

    autoAlignLeft.whileTrue(superstructure.alignLeftHoldCommand());
    autoAlignRight.whileTrue(superstructure.alignRightHoldCommand());
    // scoreIntentButton
    //     .whileTrue(new RunCommand(this::AutoScoreAlign))
    //     .onFalse(new InstantCommand(() -> aligning = false));

    elevatorTargetL1.onTrue(
        superstructure.setDeployedCoralLevelCommand(SuperstructurePose.ElevatorLevel.L1));
    elevatorTargetL2.onTrue(
        superstructure.setDeployedCoralLevelCommand(SuperstructurePose.ElevatorLevel.L2));
    elevatorTargetL3.onTrue(
        superstructure.setDeployedCoralLevelCommand(SuperstructurePose.ElevatorLevel.L3));
    elevatorTargetL4.onTrue(
        superstructure.setDeployedCoralLevelCommand(SuperstructurePose.ElevatorLevel.L4));
    elevatorDeploy.onTrue(superstructure.deployCommand());
    elevatorStow.onTrue(superstructure.stowCommand());

    // coralUnstuck.whileTrue(superstructure.coralUnstuckCommand());

    // climbToggle.onTrue(new InstantCommand(() -> state.toggleClimb(climber)));
    // climbStow.onTrue(new InstantCommand(() -> climber.setClimbSpeed(-1)))
    //          .onFalse(new InstantCommand(() -> climber.setClimbSpeed(0)));
    // funnelDrop.onTrue(state.dropFunnel(climber).andThen(new
    // InstantCommand(climber::resetFunnel)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
