package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraDuty;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.ReefSide;
import frc.robot.util.GeometryUtil;
import frc.robot.util.NTPrefixes;
import java.util.List;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static Rotation2d robotHeading;
  private static Rotation2d headingOffset;
  private static SwerveModulePosition[] modulePositions;

  private static final SwerveDrivePoseEstimator fieldLocalizer;
  private static final SwerveDrivePoseEstimator reefLocalizer;
  private static final SwerveDriveOdometry odometry;

  @Getter private static ReefAlignData reefAlignData;
  @Getter private static OperatorInputData OIData;
  @Getter private static RobotConfigurationData robotConfigurationData;

  @Getter
  @Setter
  @AutoLogOutput(key = "RobotState/ElevatorExtensionPercent")
  private double elevatorExtensionPercent;

  @Getter @Setter private static RobotMode mode;
  @Getter @Setter private static boolean hasAlgae;

  @Getter @Setter private static boolean isIntakingCoral;
  @Getter @Setter private static boolean isIntakingAlgae;
  @Getter @Setter private static boolean isAutoAligning;
  @Getter @Setter private static boolean autoClapOverride;

  static {
    OIData = new OperatorInputData(ReefSide.LEFT, ElevatorPositions.STOW);

    robotHeading = new Rotation2d();
    headingOffset = new Rotation2d();
    modulePositions = new SwerveModulePosition[4];

    for (int i = 0; i < modulePositions.length; i++) {
      modulePositions[i] = new SwerveModulePosition();
    }

    fieldLocalizer =
        new SwerveDrivePoseEstimator(
            new SwerveDriveKinematics(DriveConstants.moduleTranslations),
            new Rotation2d(),
            modulePositions,
            new Pose2d());
    reefLocalizer =
        new SwerveDrivePoseEstimator(
            new SwerveDriveKinematics(DriveConstants.moduleTranslations),
            new Rotation2d(),
            modulePositions,
            new Pose2d());
    odometry =
        new SwerveDriveOdometry(
            new SwerveDriveKinematics(DriveConstants.moduleTranslations),
            new Rotation2d(),
            modulePositions);

    reefAlignData =
        new ReefAlignData(
            -1,
            new Pose2d(),
            new Pose2d(),
            0.0,
            0.0,
            false,
            false,
            ElevatorPositions.ALGAE_INTAKE_BOT);
    robotConfigurationData = new RobotConfigurationData(0.0, new Rotation2d(), 0.0);
  }

  public RobotState() {}

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  public static void periodic(
      Rotation2d robotHeading,
      long latestRobotHeadingTimestamp,
      double robotYawVelocity,
      Translation2d robotFieldRelativeVelocity,
      SwerveModulePosition[] modulePositions,
      Camera[] cameras) {
    RobotState.robotHeading = robotHeading;
    RobotState.modulePositions = modulePositions;

    fieldLocalizer.updateWithTime(Timer.getTimestamp(), robotHeading, modulePositions);
    reefLocalizer.updateWithTime(Timer.getTimestamp(), robotHeading, modulePositions);
    odometry.update(robotHeading, modulePositions);

    for (Camera camera : cameras) {
      double[] limelightHeadingData = {
        robotHeading.minus(headingOffset).getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0
      };
      camera.getRobotHeadingPublisher().set(limelightHeadingData, latestRobotHeadingTimestamp);
    }

    NetworkTableInstance.getDefault().flush();

    for (Camera camera : cameras) {
      if (camera.getCameraDuties().contains(CameraDuty.FIELD_LOCALIZATION)
          && camera.getTargetAquired()
          && !GeometryUtil.isZero(camera.getPrimaryPose())
          && !GeometryUtil.isZero(camera.getSecondaryPose())
          && Math.abs(robotYawVelocity) <= Units.degreesToRadians(15.0)
          && Math.abs(robotFieldRelativeVelocity.getNorm()) <= 1.0
          && camera.getTotalTargets() > 0) {
        double xyStddevPrimary =
            camera.getPrimaryXYStandardDeviationCoefficient()
                * Math.pow(camera.getAverageDistance(), 2.0)
                / camera.getTotalTargets()
                * camera.getHorizontalFOV();
        fieldLocalizer.addVisionMeasurement(
            camera.getPrimaryPose(),
            camera.getFrameTimestamp(),
            VecBuilder.fill(xyStddevPrimary, xyStddevPrimary, Double.POSITIVE_INFINITY));
        if (camera.getTotalTargets() > 1) {
          double xyStddevSecondary =
              camera.getSecondaryXYStandardDeviationCoefficient()
                  * Math.pow(camera.getAverageDistance(), 2.0)
                  / camera.getTotalTargets()
                  * camera.getHorizontalFOV();
          fieldLocalizer.addVisionMeasurement(
              camera.getSecondaryPose(),
              camera.getFrameTimestamp(),
              VecBuilder.fill(xyStddevSecondary, xyStddevSecondary, Double.POSITIVE_INFINITY));
        }
      }
    }

    int closestReefTag = getMinDistanceReefTag();

    for (Camera camera : cameras) {
      if (camera.getCameraDuties().contains(CameraDuty.REEF_LOCALIZATION)
          && !GeometryUtil.isZero(camera.getPrimaryPose())
          && camera.getTotalTargets() > 0) {
        double xyStddevPrimary =
            camera.getPrimaryXYStandardDeviationCoefficient()
                * Math.pow(camera.getAverageDistance(), 2.0)
                / camera.getTotalTargets()
                * camera.getHorizontalFOV();
        reefLocalizer.addVisionMeasurement(
            camera.getPrimaryPose(),
            camera.getFrameTimestamp(),
            VecBuilder.fill(xyStddevPrimary, xyStddevPrimary, Double.POSITIVE_INFINITY));
      }
    }

    Pose2d autoAlignCoralSetpoint =
        OIData.currentReefHeight().equals(ElevatorPositions.L1)
            ? FieldConstants.getNearestReefFace(getRobotPoseReef())
            : FieldConstants.getNearestReefBranch(getRobotPoseReef(), OIData.currentReefPost());
    Pose2d autoAlignAlgaeSetpoint = FieldConstants.getNearestReefFace(getRobotPoseReef());

    double distanceToCoralSetpoint =
        RobotState.getRobotPoseReef()
            .getTranslation()
            .getDistance(autoAlignCoralSetpoint.getTranslation());
    double distanceToAlgaeSetpoint =
        RobotState.getRobotPoseReef()
            .getTranslation()
            .getDistance(autoAlignAlgaeSetpoint.getTranslation());

    boolean atCoralSetpoint =
        Math.abs(distanceToCoralSetpoint)
            <= DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.positionThresholdMeters().get();
    boolean atAlgaeSetpoint =
        Math.abs(distanceToAlgaeSetpoint)
            <= DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.positionThresholdMeters().get();

    ElevatorPositions algaeHeight;
    switch (closestReefTag) {
      case 10, 6, 8, 21, 17, 19:
        algaeHeight = ElevatorPositions.ALGAE_INTAKE_BOT;
        break;
      case 9, 11, 7, 22, 20, 18:
        algaeHeight = ElevatorPositions.ALGAE_INTAKE_TOP;
        break;
      default:
        algaeHeight = ElevatorPositions.ALGAE_INTAKE_BOT;
        break;
    }
    ;

    reefAlignData =
        new ReefAlignData(
            closestReefTag,
            autoAlignCoralSetpoint,
            autoAlignAlgaeSetpoint,
            distanceToCoralSetpoint,
            distanceToAlgaeSetpoint,
            atCoralSetpoint,
            atAlgaeSetpoint,
            algaeHeight,
            cameras);

    Logger.recordOutput(NTPrefixes.ROBOT_STATE + "Has Algae", hasAlgae);

    Logger.recordOutput(NTPrefixes.POSE_DATA + "Field Pose", fieldLocalizer.getEstimatedPosition());
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Odometry Pose", odometry.getPoseMeters());
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Heading Offset", headingOffset);

    Logger.recordOutput(NTPrefixes.OI_DATA + "Reef Post", OIData.currentReefPost());
    Logger.recordOutput(NTPrefixes.OI_DATA + "Reef Height", OIData.currentReefHeight());

    Logger.recordOutput(NTPrefixes.REEF_DATA + "Reef Pose", reefLocalizer.getEstimatedPosition());
    Logger.recordOutput(NTPrefixes.REEF_DATA + "Closest Reef April Tag", closestReefTag);

    Logger.recordOutput(NTPrefixes.CORAL_DATA + "Coral Setpoint", autoAlignCoralSetpoint);
    Logger.recordOutput(NTPrefixes.CORAL_DATA + "Coral Setpoint Error", distanceToCoralSetpoint);
    Logger.recordOutput(NTPrefixes.CORAL_DATA + "At Coral Setpoint", atCoralSetpoint);

    Logger.recordOutput(NTPrefixes.ALGAE_DATA + "Algae Setpoint", autoAlignAlgaeSetpoint);
    Logger.recordOutput(NTPrefixes.ALGAE_DATA + "Algae Setpoint Error", distanceToAlgaeSetpoint);
    Logger.recordOutput(NTPrefixes.ALGAE_DATA + "At Algae Setpoint", atAlgaeSetpoint);
  }

  public static void periodic(
      Rotation2d robotHeading,
      long latestRobotHeadingTimestamp,
      double robotYawVelocity,
      Translation2d robotFieldRelativeVelocity,
      SwerveModulePosition[] modulePositions,
      double intakeStart,
      Rotation2d armStart,
      double elevatorStart,
      Camera[] cameras) {
    RobotState.robotHeading = robotHeading;
    RobotState.modulePositions = modulePositions;

    fieldLocalizer.updateWithTime(Timer.getTimestamp(), robotHeading, modulePositions);
    reefLocalizer.updateWithTime(Timer.getTimestamp(), robotHeading, modulePositions);
    odometry.update(robotHeading, modulePositions);

    for (Camera camera : cameras) {
      double[] limelightHeadingData = {
        robotHeading.minus(headingOffset).getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0
      };
      camera.getRobotHeadingPublisher().set(limelightHeadingData, latestRobotHeadingTimestamp);
    }

    NetworkTableInstance.getDefault().flush();

    for (Camera camera : cameras) {
      if (camera.getCameraDuties().contains(CameraDuty.FIELD_LOCALIZATION)
          && camera.getTargetAquired()
          && !GeometryUtil.isZero(camera.getPrimaryPose())
          && Math.abs(robotYawVelocity) <= Units.degreesToRadians(15.0)
          && Math.abs(robotFieldRelativeVelocity.getNorm()) <= 1.0
          && camera.getTotalTargets() > 0
          && RobotMode.enabled()) {
        double xyStddevPrimary =
            camera.getPrimaryXYStandardDeviationCoefficient()
                * Math.pow(camera.getAverageDistance(), 2.0)
                / camera.getTotalTargets()
                * camera.getHorizontalFOV();
        fieldLocalizer.addVisionMeasurement(
            camera.getPrimaryPose(),
            camera.getFrameTimestamp(),
            VecBuilder.fill(xyStddevPrimary, xyStddevPrimary, Double.POSITIVE_INFINITY));
      }
      if (camera.getCameraDuties().contains(CameraDuty.FIELD_LOCALIZATION)
          && camera.getTargetAquired()
          && !GeometryUtil.isZero(camera.getSecondaryPose())
          && Math.abs(robotYawVelocity) <= Units.degreesToRadians(10.0)
          && Math.abs(robotFieldRelativeVelocity.getNorm()) <= 1.0
          && camera.getTotalTargets() > 1
          && RobotMode.disabled()) {
        double xyStddevSecondary =
            camera.getSecondaryXYStandardDeviationCoefficient()
                * Math.pow(camera.getAverageDistance(), 2.0)
                / camera.getTotalTargets()
                * camera.getHorizontalFOV();
        fieldLocalizer.addVisionMeasurement(
            camera.getSecondaryPose(),
            camera.getFrameTimestamp(),
            VecBuilder.fill(xyStddevSecondary, xyStddevSecondary, 0.1));
      }
    }

    int closestReefTag = getMinDistanceReefTag();

    for (Camera camera : cameras) {
      if (camera.getCameraDuties().contains(CameraDuty.REEF_LOCALIZATION)
          && !GeometryUtil.isZero(camera.getPrimaryPose())
          && camera.getTotalTargets() > 0) {
        double xyStddevPrimary =
            camera.getPrimaryXYStandardDeviationCoefficient()
                * Math.pow(camera.getAverageDistance(), 2.0)
                / camera.getTotalTargets()
                * camera.getHorizontalFOV();
        reefLocalizer.addVisionMeasurement(
            camera.getPrimaryPose(),
            camera.getFrameTimestamp(),
            VecBuilder.fill(xyStddevPrimary, xyStddevPrimary, Double.POSITIVE_INFINITY));
      }
    }

    if (RobotMode.disabled()) {
      resetRobotPose(getRobotPoseField());
    }

    Pose2d autoAlignCoralSetpoint =
        OIData.currentReefHeight().equals(ElevatorPositions.L1)
            ? FieldConstants.getNearestReefFace(getRobotPoseReef())
            : FieldConstants.getNearestReefBranch(getRobotPoseReef(), OIData.currentReefPost());
    Pose2d autoAlignAlgaeSetpoint = FieldConstants.getNearestReefFace(getRobotPoseReef());

    double distanceToCoralSetpoint =
        RobotState.getRobotPoseReef()
            .getTranslation()
            .getDistance(autoAlignCoralSetpoint.getTranslation());
    double distanceToAlgaeSetpoint =
        RobotState.getRobotPoseReef()
            .getTranslation()
            .getDistance(autoAlignAlgaeSetpoint.getTranslation());

    boolean atCoralSetpoint =
        Math.abs(distanceToCoralSetpoint)
            <= DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.positionThresholdMeters().get();
    boolean atAlgaeSetpoint =
        Math.abs(distanceToAlgaeSetpoint)
            <= DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.positionThresholdMeters().get();

    ElevatorPositions algaeHeight;
    switch (closestReefTag) {
      case 10:
      case 6:
      case 8:
      case 21:
      case 17:
      case 19:
        algaeHeight = ElevatorPositions.ALGAE_INTAKE_BOT;
        break;
      case 9:
      case 11:
      case 7:
      case 22:
      case 20:
      case 18:
        algaeHeight = ElevatorPositions.ALGAE_INTAKE_TOP;
        break;
      default:
        algaeHeight = ElevatorPositions.ALGAE_INTAKE_BOT;
        break;
    }

    reefAlignData =
        new ReefAlignData(
            closestReefTag,
            autoAlignCoralSetpoint,
            autoAlignAlgaeSetpoint,
            distanceToCoralSetpoint,
            distanceToAlgaeSetpoint,
            atCoralSetpoint,
            atAlgaeSetpoint,
            algaeHeight,
            cameras);

    robotConfigurationData = new RobotConfigurationData(intakeStart, armStart, elevatorStart);

    Logger.recordOutput(NTPrefixes.ROBOT_STATE + "Has Algae", hasAlgae);

    Logger.recordOutput(NTPrefixes.POSE_DATA + "Field Pose", fieldLocalizer.getEstimatedPosition());
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Odometry Pose", odometry.getPoseMeters());
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Heading Offset", headingOffset);

    Logger.recordOutput(NTPrefixes.OI_DATA + "Reef Post", OIData.currentReefPost());
    Logger.recordOutput(NTPrefixes.OI_DATA + "Reef Height", OIData.currentReefHeight());

    Logger.recordOutput(NTPrefixes.REEF_DATA + "Reef Pose", reefLocalizer.getEstimatedPosition());
    Logger.recordOutput(NTPrefixes.REEF_DATA + "Closest Reef April Tag", closestReefTag);

    Logger.recordOutput(NTPrefixes.CORAL_DATA + "Coral Setpoint", autoAlignCoralSetpoint);
    Logger.recordOutput(NTPrefixes.CORAL_DATA + "Coral Setpoint Error", distanceToCoralSetpoint);
    Logger.recordOutput(NTPrefixes.CORAL_DATA + "At Coral Setpoint", atCoralSetpoint);

    Logger.recordOutput(NTPrefixes.ALGAE_DATA + "Algae Setpoint", autoAlignAlgaeSetpoint);
    Logger.recordOutput(NTPrefixes.ALGAE_DATA + "Algae Setpoint Error", distanceToAlgaeSetpoint);
    Logger.recordOutput(NTPrefixes.ALGAE_DATA + "At Algae Setpoint", atAlgaeSetpoint);
    Logger.recordOutput(NTPrefixes.ALGAE_DATA + "Algae Height", algaeHeight);
  }

  public static Pose2d getRobotPoseField() {
    return fieldLocalizer.getEstimatedPosition();
  }

  public static Pose2d getRobotPoseReef() {
    return reefLocalizer.getEstimatedPosition();
  }

  public static Pose2d getRobotPoseOdometry() {
    return odometry.getPoseMeters();
  }

  private static int getMinDistanceReefTag() {
    Pose2d reefFace = FieldConstants.getNearestReefFace(RobotState.getRobotPoseReef());
    int minDistanceTag = List.of(FieldConstants.Reef.centerFaces).indexOf(reefFace);

    if (AllianceFlipUtil.shouldFlip()) {
      switch (minDistanceTag) {
        case 0:
          minDistanceTag = 7;
          break;
        case 1:
          minDistanceTag = 6;
          break;
        case 2:
          minDistanceTag = 11;
          break;
        case 3:
          minDistanceTag = 10;
          break;
        case 4:
          minDistanceTag = 9;
          break;
        case 5:
          minDistanceTag = 8;
          break;
      }
    } else {
      switch (minDistanceTag) {
        case 0:
          minDistanceTag = 18;
          break;
        case 1:
          minDistanceTag = 19;
          break;
        case 2:
          minDistanceTag = 20;
          break;
        case 3:
          minDistanceTag = 21;
          break;
        case 4:
          minDistanceTag = 22;
          break;
        case 5:
          minDistanceTag = 17;
          break;
      }
    }
    return minDistanceTag;
  }

  public static void resetRobotPose(Pose2d pose) {
    headingOffset = robotHeading.minus(pose.getRotation());
    fieldLocalizer.resetPosition(robotHeading, modulePositions, pose);
    reefLocalizer.resetPosition(robotHeading, modulePositions, pose);
    odometry.resetPosition(robotHeading, modulePositions, pose);
  }

  public static void setReefPost(ReefSide post) {
    ElevatorPositions height = OIData.currentReefHeight();
    OIData = new OperatorInputData(post, height);
  }

  public static void toggleReefPost() {
    ElevatorPositions height = OIData.currentReefHeight();
    if (OIData.currentReefPost().equals(ReefSide.LEFT)) {
      OIData = new OperatorInputData(ReefSide.RIGHT, height);
    } else {
      OIData = new OperatorInputData(ReefSide.LEFT, height);
    }
  }

  public static void setReefHeight(ElevatorPositions height) {
    ReefSide post = OIData.currentReefPost();
    OIData = new OperatorInputData(post, height);
  }

  public static final record ReefAlignData(
      int closestReefTag,
      Pose2d coralSetpoint,
      Pose2d algaeSetpoint,
      double distanceToCoralSetpoint,
      double distanceToAlgaeSetpoint,
      boolean atCoralSetpoint,
      boolean atAlgaeSetpoint,
      ElevatorPositions algaeIntakeHeight,
      Camera... cameras) {}

  public static final record OperatorInputData(
      ReefSide currentReefPost, ElevatorPositions currentReefHeight) {}

  public static final record RobotConfigurationData(
      double intakeStart, Rotation2d armStart, double elevatorStart) {}

  public enum RobotMode {
    DISABLED,
    TELEOP,
    AUTO;

    public static boolean enabled(RobotMode mode) {
      return mode.equals(TELEOP) || mode.equals(AUTO);
    }

    public static boolean disabled(RobotMode mode) {
      return mode.equals(DISABLED);
    }

    public static boolean teleop(RobotMode mode) {
      return mode.equals(TELEOP);
    }

    public static boolean auto(RobotMode mode) {
      return mode.equals(AUTO);
    }

    public static boolean enabled() {
      return enabled(RobotState.getMode());
    }

    public static boolean disabled() {
      return disabled(RobotState.getMode());
    }

    public static boolean teleop() {
      return teleop(RobotState.getMode());
    }

    public static boolean auto() {
      return auto(RobotState.getMode());
    }
  }
}
