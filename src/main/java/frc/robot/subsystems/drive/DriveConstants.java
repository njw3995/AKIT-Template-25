// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.swerve.ModuleLimits;
import lombok.Builder;

public class DriveConstants {
  // Odometry frequency now depends on CAN FD support (like the PP template)
  public static final double odometryFrequency =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;

  // Keep public names; compute from Tuner geometry
  public static final Translation2d[] moduleTranslations = {
    new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
    new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
    new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
    new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
  };

  // Preserve the trackWidth names for any existing users (derived from translations)
  public static final double trackWidthX =
      Math.abs(maxX(moduleTranslations) - minX(moduleTranslations));
  public static final double trackWidthY =
      Math.abs(maxY(moduleTranslations) - minY(moduleTranslations));

  public static final double driveBaseRadius =
      Math.max(
          Math.max(moduleTranslations[0].getNorm(), moduleTranslations[1].getNorm()),
          Math.max(moduleTranslations[2].getNorm(), moduleTranslations[3].getNorm()));

  // Max speeds now come from TunerConstants
  public static final double maxLinearSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public static final double maxAngularSpeed = maxLinearSpeed / driveBaseRadius;
  public static final double maxLinearAcceleration = 22.0;

  /** Includes bumpers! */
  public static final double robotWidth =
      Units.inchesToMeters(28.0) + 2 * Units.inchesToMeters(2.0);

  // Wheel radius from Tuner
  public static final double wheelRadius = TunerConstants.FrontLeft.WheelRadius;

  public static final ModuleLimits moduleLimitsFree =
      new ModuleLimits(maxLinearSpeed, maxLinearAcceleration, Units.degreesToRadians(1080.0));

  public static final AlignRobotToAprilTagConstants ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS;

  public static final ModuleConfig[] moduleConfigsComp = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(16)
        .turnMotorId(15)
        .encoderChannel(41)
        .encoderOffset(Rotation2d.fromRadians(2.5356702423749646))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(10)
        .turnMotorId(11)
        .encoderChannel(42)
        .encoderOffset(Rotation2d.fromRadians(-2.932971266437346))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(18)
        .turnMotorId(19)
        .encoderChannel(43)
        .encoderOffset(Rotation2d.fromRadians(0.6458059116998549))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(13)
        .turnMotorId(14)
        .encoderChannel(44)
        .encoderOffset(Rotation2d.fromRadians(-2.5187964537082226))
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  public static final ModuleConfig[] moduleConfigsDev = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(12)
        .turnMotorId(9)
        .encoderChannel(1)
        .encoderOffset(Rotation2d.fromRadians(-0.009115335014721037))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(2)
        .turnMotorId(10)
        .encoderChannel(3)
        .encoderOffset(Rotation2d.fromRadians(0.8427416931125384))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(15)
        .turnMotorId(11)
        .encoderChannel(0)
        .encoderOffset(Rotation2d.fromRadians(-1.0620197413817225))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(3)
        .turnMotorId(8)
        .encoderChannel(2)
        .encoderOffset(Rotation2d.fromRadians(-2.600063124240756))
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  public static class PigeonConstants {
    public static final int id = Constants.getRobot() == RobotType.DEVBOT ? 3 : 30;
  }

  @Builder
  public record ModuleConfig(
      int driveMotorId,
      int turnMotorId,
      int encoderChannel,
      Rotation2d encoderOffset,
      boolean turnInverted,
      boolean encoderInverted) {}

  public record PIDControllerConstants(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber tolerance,
      LoggedTunableNumber maxVelocity) {}

  public static record AlignRobotToAprilTagConstants(
      PIDControllerConstants xPIDConstants,
      PIDControllerConstants yPIDConstants,
      PIDControllerConstants omegaPIDConstants,
      LoggedTunableNumber positionThresholdMeters) {}

  static {
    ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS =
        new AlignRobotToAprilTagConstants(
            new PIDControllerConstants(
                new LoggedTunableNumber("Drive/Align Robot To April Tag/X Constants/kP", 3),
                new LoggedTunableNumber("Drive/Align Robot To April Tag/X Constants/kD", 0.15),
                new LoggedTunableNumber(
                    "Drive/Align Robot To April Tag/X Constants/tolerance", 0.03),
                new LoggedTunableNumber(
                    "Drive/Align Robot To April Tag/X Constants/maxVelocity", 2.5)),
            new PIDControllerConstants(
                new LoggedTunableNumber("Drive/Align Robot To April Tag/Y Constants/kP", 3),
                new LoggedTunableNumber("Drive/Align Robot To April Tag/Y Constants/kD", 0.15),
                new LoggedTunableNumber(
                    "Drive/Align Robot To April Tag/Y Constants/tolerance", 0.03),
                new LoggedTunableNumber(
                    "Drive/Align Robot To April Tag/Y Constants/maxVelocity", 2.5)),
            new PIDControllerConstants(
                new LoggedTunableNumber(
                    "Drive/Align Robot To April Tag/Omega Constants/kP", 2 * Math.PI),
                new LoggedTunableNumber("Drive/Align Robot To April Tag/Omega Constants/kD", 0.05),
                new LoggedTunableNumber(
                    "Drive/Align Robot To April Tag/Omega Constants/tolerance",
                    Units.degreesToRadians(0.25)),
                new LoggedTunableNumber(
                    "Drive/Align Robot To April Tag/Omega Constants/maxVelocity", Math.PI)),
            new LoggedTunableNumber(
                "Drive/Align Robot To April Tag/positionThresholdDegrees", 0.03));
  }

  // --- helpers for track width calculations ---
  private static double minX(Translation2d[] m) {
    double v = Double.POSITIVE_INFINITY;
    for (var t : m) v = Math.min(v, t.getX());
    return v;
  }

  private static double maxX(Translation2d[] m) {
    double v = Double.NEGATIVE_INFINITY;
    for (var t : m) v = Math.max(v, t.getX());
    return v;
  }

  private static double minY(Translation2d[] m) {
    double v = Double.POSITIVE_INFINITY;
    for (var t : m) v = Math.min(v, t.getY());
    return v;
  }

  private static double maxY(Translation2d[] m) {
    double v = Double.NEGATIVE_INFINITY;
    for (var t : m) v = Math.max(v, t.getY());
    return v;
  }
}
