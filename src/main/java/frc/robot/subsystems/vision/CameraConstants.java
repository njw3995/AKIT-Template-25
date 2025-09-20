package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;

/**
 * Constants and helpers for camera configuration (FOV, std-dev coefficients, and robot camera
 * list).
 */
public class CameraConstants {
  /** Approximate LED blink period in seconds (utility for UI/debug). */
  public static final double BLINK_TIME = 0.067;

  /** Limelight 2+ nominal parameters. */
  public static class Limelight2PlusConstants {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(62.5);
    public static final double VERTICAL_FOV = Units.degreesToRadians(48.9);
    public static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    public static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    public static final double COMPLEMENTARY_FILTER_SIGMA = 0.5;
  }

  /** Limelight 3 nominal parameters. */
  public static class Limelight3Constants {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(62.5);
    public static final double VERTICAL_FOV = Units.degreesToRadians(48.9);
    public static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    public static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
  }

  /** Limelight 3G nominal parameters. */
  public static class Limelight3GConstants {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(82.0);
    public static final double VERTICAL_FOV = Units.degreesToRadians(46.2);
    public static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.05;
    public static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.00015;
  }

  /** Limelight 4 nominal parameters. */
  public static class Limelight4Constants {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(82.0);
    public static final double VERTICAL_FOV = Units.degreesToRadians(46.2);
    public static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.05;
    public static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.00015;
  }

  /** Factory for robot cameras. Edit transforms and duties to match your robot configuration. */
  public static class RobotCameras {
    /** Left-side Limelight configuration including physical transform and duties. */
    private static final Camera LEFT =
        new Camera(
            new CameraIOLimelight("left", CameraType.LIMELIGHT_3),
            Limelight3GConstants.HORIZONTAL_FOV,
            Limelight3GConstants.VERTICAL_FOV,
            Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault()
                .getTable("limelight-left")
                .getDoubleArrayTopic("robot_orientation_set")
                .publish(),
            List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION),
            new Transform3d(
                -0.204072,
                -0.280928,
                0.231125,
                new Rotation3d(0.0, 0, Units.degreesToRadians(225))));

    /** Right-side Limelight configuration including physical transform and duties. */
    private static final Camera RIGHT =
        new Camera(
            new CameraIOLimelight("right", CameraType.LIMELIGHT_3),
            Limelight4Constants.HORIZONTAL_FOV,
            Limelight4Constants.VERTICAL_FOV,
            Limelight4Constants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight4Constants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault()
                .getTable("limelight-right")
                .getDoubleArrayTopic("robot_orientation_set")
                .publish(),
            List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION),
            new Transform3d(
                -0.204072, 0.280928, 0.231125, new Rotation3d(0, 0, Units.degreesToRadians(-225))));

    /** Cameras used by the robot. Order is not significant. */
    public static final Camera[] CAMERAS = {LEFT, RIGHT};
  }

  /** Placeholder for replay/sim configurations if needed. */
  public static class ReplayCameras {}
}
