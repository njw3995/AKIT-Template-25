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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static final Mode simMode = Mode.SIM;
  private static RobotType robotType = RobotType.COMPBOT;
  public static final boolean tuningMode = false;

  @SuppressWarnings("resource")
  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.kError)
          .set(true);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (robotType) {
      case DEVBOT, COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    SIMBOT,
    DEVBOT,
    COMPBOT
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (robotType == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + robotType);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (robotType != RobotType.COMPBOT || tuningMode) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }

  public static final class CanIDS {

    public static final int ELEVATOR_LEADER_CAN_ID = 1;
    public static final int ELEVATOR_FOLLOWER_CAN_ID = 2;

    public static final int WRIST_CAN_ID = 3;
    public static final int WRIST_ENCODER_CAN_ID = 6; // Or analog id?

    public static final int END_EFFECTOR_CAN_ID = 4;

    public static final int CLIMBER_CAN_ID = 5;
    public static final int CLIMBER_ENCODER_CAN_ID = 7; // or analog id
  }

  public static final class DIOPorts {

    public static final int ELEVATOR_LOWER_LIMIT_DIO_PORT =
        8; // Was previously plugged directly into flex, kraken cant do that :(
    public static final int ELEVATOR_ZERO_LIMIT_DIO_PORT = 9;

    public static final int END_EFFECTOR_INTAKE_CORAL_BEAM_BREAK_DIO_PORT = 8; // Or analog id?
    public static final int END_EFFECTOR_PLACEMENT_CORAL_BEAM_BREAK_DIO_PORT = 7; // Or analog id?
  }

  public static class MotorConstants {
    public static class KrakenConstants {
      public static final double MAX_RPM = 6000.0;
      public static final double NOMINAL_VOLTAGE_VOLTS = 12.0;
      public static final double STALL_TORQUE_NEWTON_METERS = 7.09;
      public static final double STALL_CURRENT_AMPS = 40.0;
      public static final double FREE_CURRENT_AMPS = 30.0;
      public static final double FREE_SPEED_RPM = 6000.0;
      public static final double SUPPLY_VOLTAGE_TIME = 0.02;
    }
  }

  public static final class AutoConstants {
    public static final double INTAKE_CORAL_WAIT_TIME = 0.25;
    public static final double INTAKE_ALGAE_WAIT_TIME = 0.5;
    public static final double SCORE_CORAL_WAIT_TIME = 0.5;
    public static final double SCORE_ALGAE_WAIT_TIME = 0.5;
    public static final double ELEVATOR_SCALER = 0.85;
    public static final double ELEVATOR_WAIT_TIME = 0.5;
    public static final double AUTO_ELEVATOR_TOLERANCE_INCHES = 1;
    public static final double AUTO_WRIST_TOLERANCE_INCHES = 2.0;
    public static final double WRIST_WAIT_TIME = 0.5;
    public static final double REEF_BRANCH_TOLERANCE = Units.inchesToMeters(1.0);
    public static final double AUTO_DROP_INDICATOR_TIMEOUT = 3;
  }
}
