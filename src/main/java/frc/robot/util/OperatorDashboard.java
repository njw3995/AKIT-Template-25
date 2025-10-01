package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.superstructure.SuperstructureState.AutomationLevel;

/** Simple Elastic/SmartDashboard chooser (no Shuffleboard). */
public class OperatorDashboard {
  private final SendableChooser<AutomationLevel> automationChooser = new SendableChooser<>();

  public OperatorDashboard() {
    automationChooser.setDefaultOption("Manual", AutomationLevel.MANUAL);
    automationChooser.addOption("Auto Release", AutomationLevel.AUTO_RELEASE);
    automationChooser.addOption(
        "Auto Drive + Manual Release", AutomationLevel.AUTO_DRIVE_AND_MANUAL_RELEASE);

    SmartDashboard.putData("Automation Level", automationChooser);
  }

  public AutomationLevel getAutomationLevel() {
    return automationChooser.getSelected();
  }
}
