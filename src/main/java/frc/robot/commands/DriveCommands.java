package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.LoggedTunableNumber;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  public static final double deadband = 0.1;
  private static final double ffStartDelay = 2.0; // Secs
  private static final double ffRampRate = 0.1; // Volts/Sec
  private static final double wheelRadiusMaxVelocity = 0.25; // Rad/Sec
  private static final double wheelRadiusRampRate = 0.05; // Rad/Sec^2

  // If you later wire elevator extension into commands, keep these tunables:
  private static final LoggedTunableNumber elevatorMinExtension =
      new LoggedTunableNumber("DriveCommands/ElevatorMinExtension", 0.4);
  private static final LoggedTunableNumber maxExtensionAngularVelocityScalar =
      new LoggedTunableNumber("DriveCommands/MaxExtensionAngularVelocityScalar", 0.2);

  private DriveCommands() {}

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), deadband);
    Rotation2d linearDirection = new Rotation2d(x, y);
    linearMagnitude = linearMagnitude * linearMagnitude; // square for finesse
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  public static double getOmegaFromJoysticks(double driverOmega) {
    double omega = MathUtil.applyDeadband(driverOmega, deadband);
    return omega * omega * Math.signum(omega);
  }

  /** Old helper preserved: assumes no elevator scaling. */
  public static ChassisSpeeds getSpeedsFromJoysticks(
      double driverX, double driverY, double driverOmega) {
    return getSpeedsFromJoysticks(driverX, driverY, driverOmega, () -> 0.0);
  }

  /** New helper that accepts elevator extension percent [0..1] to scale omega if desired. */
  public static ChassisSpeeds getSpeedsFromJoysticks(
      double driverX, double driverY, double driverOmega, DoubleSupplier elevatorExtensionPercent) {
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(driverX, driverY).times(DriveConstants.maxLinearSpeed);

    double omegaNorm = getOmegaFromJoysticks(driverOmega);

    // Angular scaling vs. extension
    double ext = MathUtil.clamp(elevatorExtensionPercent.getAsDouble(), 0.0, 1.0);
    double extNorm =
        MathUtil.clamp(
            (ext - elevatorMinExtension.get()) / Math.max(1e-9, (1.0 - elevatorMinExtension.get())),
            0.0,
            1.0);
    double omegaScale = MathUtil.interpolate(1.0, maxExtensionAngularVelocityScalar.get(), extNorm);

    return new ChassisSpeeds(
        linearVelocity.getX(),
        linearVelocity.getY(),
        omegaNorm * DriveConstants.maxAngularSpeed * omegaScale);
  }

  /** Old command preserved: robotRelative only, no elevator scaling. */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier robotRelative) {
    return joystickDrive(drive, xSupplier, ySupplier, omegaSupplier, robotRelative, () -> 0.0);
  }

  /** New command that includes elevator extension scaling hook. */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier robotRelative,
      DoubleSupplier elevatorExtensionPercent) {
    return Commands.run(
        () -> {
          ChassisSpeeds speeds =
              getSpeedsFromJoysticks(
                  xSupplier.getAsDouble(),
                  ySupplier.getAsDouble(),
                  omegaSupplier.getAsDouble(),
                  elevatorExtensionPercent);

          var fieldRot = RobotState.getInstance().getRobotPoseField().getRotation();
          // Red alliance flips field frame
          if (DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red) {
            fieldRot = fieldRot.plus(Rotation2d.kPi);
          }

          drive.runVelocity(
              robotRelative.getAsBoolean()
                  ? speeds
                  : ChassisSpeeds.fromFieldRelativeSpeeds(speeds, fieldRot));
        },
        drive);
  }

  /** Measures the velocity feedforward constants for the drive motors. */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),
        Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(ffStartDelay),
        Commands.runOnce(timer::restart),
        Commands.run(
                () -> {
                  double voltage = timer.get() * ffRampRate;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    double x = velocitySamples.get(i);
                    double y = voltageSamples.get(i);
                    sumX += x;
                    sumY += y;
                    sumXY += x * y;
                    sumX2 += x * x;
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(wheelRadiusRampRate);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        Commands.sequence(
            Commands.runOnce(() -> limiter.reset(0.0)),
            Commands.run(
                () -> {
                  double speed = limiter.calculate(wheelRadiusMaxVelocity);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),
        Commands.sequence(
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getGyroRotation();
                  state.gyroDelta = 0.0;
                }),
            Commands.run(
                    () -> {
                      var rotation = drive.getGyroRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;

                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      Logger.recordOutput("Drive/WheelDelta", wheelDelta);
                      Logger.recordOutput("Drive/WheelRadius", wheelRadius);
                    })
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000000000000000000000000000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
