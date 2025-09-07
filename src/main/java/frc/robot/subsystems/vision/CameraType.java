package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.CameraConstants.Limelight2PlusConstants;
import frc.robot.subsystems.vision.CameraConstants.Limelight3Constants;
import frc.robot.subsystems.vision.CameraConstants.Limelight3GConstants;
import frc.robot.subsystems.vision.CameraConstants.Limelight4Constants;

public enum CameraType {
  LIMELIGHT_2_PLUS(
      Limelight2PlusConstants.HORIZONTAL_FOV,
      Limelight2PlusConstants.VERTICAL_FOV,
      Limelight2PlusConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight2PlusConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT),
  LIMELIGHT_3(
      Limelight3Constants.HORIZONTAL_FOV,
      Limelight3Constants.VERTICAL_FOV,
      Limelight3Constants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight3Constants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT),
  LIMELIGHT_3G(
      Limelight3GConstants.HORIZONTAL_FOV,
      Limelight3GConstants.VERTICAL_FOV,
      Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT),
  LIMELIGHT_4(
      Limelight4Constants.HORIZONTAL_FOV,
      Limelight4Constants.VERTICAL_FOV,
      Limelight4Constants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight4Constants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT),
  DEFAULT();

  public final double horizontalFOV;
  public final double verticalFOV;
  public final double primaryXYStandardDeviationCoefficient;
  public final double secondaryXYStandardDeviationCoefficient;

  private CameraType(
      double horizontalFOV,
      double verticalFOV,
      double primaryXYStandardDeviationCoefficient,
      double secondaryXYStandardDeviationCoefficient) {
    this.horizontalFOV = horizontalFOV;
    this.verticalFOV = verticalFOV;
    this.primaryXYStandardDeviationCoefficient = primaryXYStandardDeviationCoefficient;
    this.secondaryXYStandardDeviationCoefficient = secondaryXYStandardDeviationCoefficient;
  }

  private CameraType(
      double horizontalFOV, double verticalFOV, double xyStandardDeviationCoefficient) {
    this.horizontalFOV = horizontalFOV;
    this.verticalFOV = verticalFOV;
    this.primaryXYStandardDeviationCoefficient = xyStandardDeviationCoefficient;
    this.secondaryXYStandardDeviationCoefficient = xyStandardDeviationCoefficient;
  }

  private CameraType() {
    this.horizontalFOV = 0.0;
    this.verticalFOV = 0.0;
    this.primaryXYStandardDeviationCoefficient = 0.0;
    this.secondaryXYStandardDeviationCoefficient = 0.0;
  }
}
