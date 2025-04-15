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

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.RangeSensorSubsystem.CANrangeConstants;
import frc.robot.subsystems.lights.Lights.Color;
import frc.robot.subsystems.lights.Lights.Segment;
import java.util.ArrayList;
import java.util.List;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final double loopPeriodSecs = 0.02;
  public static final boolean bypassReefDetection = true;
  public static final boolean tuningMode = false;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  public static class AutonStartPositions {
    // Measured from the center of the ice cream
    public static final double separation = Units.inchesToMeters(72.0);
    public static final Pose2d right =
        new Pose2d(Meters.of(7.26), Meters.of(2.65), new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d middle =
        new Pose2d(right.getMeasureX(), Meters.of(4.16), right.getRotation());
    public static final Pose2d left =
        new Pose2d(right.getMeasureX(), Meters.of(5.60), right.getRotation());
    public static final double TRANSLATION_START_ERROR = Units.inchesToMeters(2);
    public static final double ROTATION_START_ERROR = 3;
  }

  public static final class AUTO {
    // algae autons
    public static final double H4_GH_PATH_DELAY = 0.3;
    // processor
    public static final double GH_PROC_ELEVATOR_DELAY = 0.8;
    public static final double PROC_EF_ELEVATOR_DELAY = 0.1;
    public static final double EF_PROC_ELEVATOR_DELAY = 0.6;
    // net
    public static final double GH_NET_ELEVATOR_DELAY = 0.3;
    public static final double IJ_NET_ELEVATOR_DELAY = 0.8;
    public static final double EF_NET_ELEVATOR_DELAY = 2;
    // coral autons
    public static final double START_ELEVATOR_DELAY = .7;
    public static final double CORAL_STATION_WAIT = 0;
    public static final double REEF_TIMEOUT = 0.4;
    public static final double REEF_POST_TIMEOUT = 0.3;
    public static final double INDEX_DELAY = 0.4;
  }

  public static final class ROLLER {
    public static final double MAX_SPEED = 100;
    public static final double MAX_VOLTAGE = 12;
  }

  public static final class INTAKE_SHOOTER {
    public static final int CANID = 30;
    public static final String CANBUS = "rio";
    public static final double MAX_SPEED = 100; // rps
    // coral
    public static final double CORAL_INTAKE_SPEED = 30;
    public static final double CORAL_INTAKE_INDEX_SPEED = 7; // speed to find back edge
    public static final double CORAL_INTAKE_INDEX_REVERSE = 3; // revolutions to reverse
    public static final double CORAL_SHOOT_SPEED = 30; // 40
    public static final double CORAL_L1_SHOOT_SPEED = -10; // 40
    public static final double CORAL_SHOOT_TIMEOUT = .3;
    public static final double CORAL_L1_SHOOT_TIMEOUT = 1;
    public static final double CORAL_AUTO_SHOOT_DELAY = 0.15;
    // algae
    public static final double ALGAE_INTAKE_SPEED = -40;
    public static final double ALGAE_HOLD_SPEED = -5;
    public static final double ALGAE_SHOOT_SPEED = 20;
    public static final double ALGAE_NET_SHOOT_SPEED = 80;
    public static final double ALGAE_NET_SHOOT_DELAY = .17; // .13;
    public static final double ALGAE_SHOOT_TIMEOUT = .2;
    //
    public static final double BUMP_VALUE = 1; // rotations
    public static final Slot0Configs SLOT0_CONFIGS =
        new Slot0Configs().withKP(.2).withKI(0).withKD(0).withKS(0).withKV(0.13).withKA(0);
    public static final MotionMagicConfigs MOTIONMAGIC_CONFIGS =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(20)
            .withMotionMagicAcceleration(40)
            .withMotionMagicJerk(400);
  }

  public static final class INTAKE_SHOOTER_LOW {
    public static final int CANID = 38;
    public static final String CANBUS = "rio";
    public static final double MAX_SPEED = 100; // rps

    // algae
    public static final double ALGAE_INTAKE_SPEED = -40;
    public static final double ALGAE_HOLD_SPEED = -5;
    public static final double ALGAE_SHOOT_SPEED = 20;
    public static final double ALGAE_NET_SHOOT_SPEED = 80;
    //
    public static final double BUMP_VALUE = 1; // rotations
    public static final Slot0Configs SLOT0_CONFIGS =
        new Slot0Configs().withKP(.2).withKI(0).withKD(0).withKS(0).withKV(0.13).withKA(0);
    public static final MotionMagicConfigs MOTIONMAGIC_CONFIGS =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(20)
            .withMotionMagicAcceleration(40)
            .withMotionMagicJerk(400);
  }

  public static final class INTAKE_CORAL_SENSOR {
    public static final CANrangeConstants CONFIGURATION_CONFIGS =
        new CANrangeConstants(
            "Coral Sensor",
            37,
            "rio",
            new Segment(2, 1, 0),
            new CANrangeConfiguration()
                .withFovParams(
                    new FovParamsConfigs()
                        .withFOVCenterX(0)
                        .withFOVCenterY(0)
                        .withFOVRangeX(6.75)
                        .withFOVRangeY(6.75))
                .withProximityParams(
                    new ProximityParamsConfigs()
                        .withProximityThreshold(400 / 1000)
                        .withProximityHysteresis(10 / 1000)
                        .withMinSignalStrengthForValidMeasurement(2500))
                .withToFParams(
                    new ToFParamsConfigs()
                        .withUpdateMode(UpdateModeValue.ShortRange100Hz)
                        .withUpdateFrequency(50)));
  }

  public static final class WRIST {
    public static final double REDUCTION = 62.5;
    public static final double MIN_POSITION = 0;
    public static final double MIN_SLOT1_TO_ELEVATE = 0.059; // .0606;
    public static final double SLOT1_TO_ELEVATE = 0.0672; // packaged position to lift elevator
    public static final double MAX_SLOT1_TO_ELEVATE = 0.075; // .0688;
    public static final double MIN_POSITION_TO_CLEAR_ELEVATOR = 0.232; // min slot2
    public static final double MAX_POSITION_AT_ELEVATOR_MIN = 0.416; // max slot2
    public static final double MAX_POSITION_AT_P1 = 0.544;
    public static final double MAX_POSITION = 0.544;
    public static final double INTAKE = 0.012; // 0.02; // 1.5 / REDUCTION;
    public static final double L1 = 0.44;
    public static final double L2 = 0.0672; // 3.9; // DO NOT MODIFY WITHOUT CHANGING
    public static final double L3 = 0.0672; // 3.9; // SEQUENCING LOGIC
    public static final double L4 = 0.0672; // 3.9; // all 3 of these must be the same value
    public static final double CRADLE = 0.192;
    public static final double SHOOTNET = 0.08;
    public static final double PRENET = 0.352;
    public static final double A2 = 0.44;
    public static final double A1 = 0.44;
    public static final double P1 = 0.528;
    public static final double BUMP_VALUE = 0.008; // rotations
    public static final double SAFE = 0.232; // minimum position to move full elevator travel
    public static final double ERROR = 0.008;
  }

  public static final class ELEVATOR {
    public static final double MAX_POSITION = 59.41; // 26.5; // MAX_BLOCK6
    public static final double MAX_POSITION_BLOCK5 = 52.81; // ????
    public static final double MAX_POSITION_BLOCK4 = 46.21; // ????
    public static final double SAFE_IN_BLOCK4 = 27.5;
    public static final double MIN_POSITION_BLOCK4 = 23.1; // ????
    public static final double MAX_POSITION_BLOCK2 = 11.22; // ????
    public static final double MIN_POSITION_AT_P1 = 7.48; // MIN_BLOCK2
    public static final double MAX_POSITION_BLOCK0 = 2.2;
    public static final double MIN_POSITION = 0; // /MIN_BLOCK0
    public static final double SHOOTNET = 58.31;
    public static final double L4 = 50.61;
    public static final double PRENET = 58.31;
    public static final double A2 = 34.11;
    public static final double L3 = 26.4;
    public static final double A1 = 19.8;
    public static final double L2 = 9.9;
    public static final double P1 = 7.48;
    public static final double L1 = 17.6;
    public static final double L1_FINAL = 8.8; // used for pivoting L1 shoot
    public static final double INTAKE = 0;
    public static final double BUMP_VALUE = 1.1; // rotations
    // sequencing contants
    public static final double MAX_LOW_SAFE = 2.2;
    public static final double MIN_MID_SAFE = 25.92;
    public static final double MAX_MID_SAFE = 35.21;
    public static final double MIN_HIGH_SAFE = 52.81;
    public static final double MAX_LOW_WRIST_MOVE_FROM_SAFE =
        6.6; // height to start wrist move if wrist is in safe position
  }

  public static final class REEFPOSTSENSOR {
    public static final CANrangeConstants CONFIGURATION_CONFIGS =
        new CANrangeConstants(
            "Reef Post Sensor",
            47,
            "rio",
            new Segment(3, 1, 0),
            new CANrangeConfiguration()
                .withFovParams(
                    new FovParamsConfigs()
                        .withFOVCenterX(0)
                        .withFOVCenterY(0)
                        .withFOVRangeX(6.75)
                        .withFOVRangeY(6.75))
                .withProximityParams(
                    new ProximityParamsConfigs()
                        .withProximityThreshold(400 / 1000)
                        .withProximityHysteresis(10 / 1000)
                        .withMinSignalStrengthForValidMeasurement(2500))
                .withToFParams(
                    new ToFParamsConfigs()
                        .withUpdateMode(UpdateModeValue.LongRangeUserFreq)
                        .withUpdateFrequency(25)));
    // must be up against reef for these limits
    public static final double LOW_LIMIT = 100; // mm
    public static final double HIGH_LIMIT = 340; // mm
  }

  public static final class CLIMBER {
    public static final int CANID = 40;
    public static final String CANBUS = "rio";
    public static final double DEPLOY = 51;
    public static final double RETRACT = -58;
    // public static final double MAX_HEIGHT = 60;
    public static final double BUMP_VALUE = 2;
    public static final double ZEROING_SPEED = -70; // speed when zeroing
    public static final double ZEROING_CURRENT_LIMIT = 45; // current limit when zeroing
    public static final Slot0Configs SLOT0_CONFIGS =
        new Slot0Configs().withKP(10).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    public static final MotionMagicConfigs MOTIONMAGIC_CONFIGS =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(70)
            .withMotionMagicAcceleration(140)
            .withMotionMagicJerk(500);
  }

  public static final class LEDSegment {
    // standard segments
    public static final Segment LEFT_SIDE = new Segment(8, 37, 1);
    public static final Segment MIDDLE = new Segment(45, 18, 2);
    public static final Segment RIGHT_SIDE = new Segment(63, 36, 3);
    public static final Segment ALL = new Segment(8, 92, 4);

    // auton setup
    public static final Segment AUTON_Y_LEFT = new Segment(45, 4, 0);
    public static final Segment AUTON_R_LEFT = new Segment(49, 3, 0);
    public static final Segment AUTON_X_LEFT = new Segment(52, 2, 0);
    public static final Segment AUTON_X_RIGHT = new Segment(54, 2, 0);
    public static final Segment AUTON_R_RIGHT = new Segment(56, 3, 0);
    public static final Segment AUTON_Y_RIGHT = new Segment(59, 4, 0);
  }

  public static final class LEDColor {
    // team colors
    public static final Color ORANGE = new Color(255, 25, 0, 0);
    public static final Color BLACK = new Color(0, 0, 0, 0);

    // indicator colors
    public static final Color WHITE = new Color(255, 230, 220, 0);
    public static final Color GREEN = new Color(56, 209, 0, 0);
    public static final Color BLUE = new Color(0, 0, 255, 0);
    public static final Color RED = new Color(255, 0, 0, 0);

    // game piece colors
    public static final Color YELLOW = new Color(242, 60, 0, 0);
    public static final Color PURPLE = new Color(184, 0, 185, 0);
  }

  // Aim constants
  public static final double X_REEF_ALIGNMENT_P = 2.5;
  public static final double Y_REEF_ALIGNMENT_P = 4.5;
  public static final double ROT_REEF_ALIGNMENT_P = 0.058;

  public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0; // Rotation
  public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0.5;
  public static final double X_SETPOINT_REEF_ALIGNMENT = -0.3; // -0.5; // Vertical pose
  public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.005;
  public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.18; // 0.19; // Horizontal pose
  public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.005;

  public static final double DONT_SEE_TAG_WAIT_TIME = 1;
  public static final double POSE_VALIDATION_TIME = 0.3;

  public static final class PathPlannerConstants {

    public static final PathConstraints testingConstraints =
        new PathConstraints(.5, 2.0, Units.degreesToRadians(50), Units.degreesToRadians(300));

    public static final PathConstraints slowConstraints =
        new PathConstraints(1.5, 4.0, Units.degreesToRadians(100), Units.degreesToRadians(720));

    public static final PathConstraints defaultConstraints =
        new PathConstraints(2, 4.0, Units.degreesToRadians(200), Units.degreesToRadians(720));

    public static final PathConstraints fastConstraints =
        new PathConstraints(3, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(720));
  }

  public static final class FieldPoses {

    public static final double[] fieldSize = {17.55, 8.05};
    // Wall thickness is 0.051
    public static final double[] centerOfReef = {4.487, 4.025};

    public static final List<Pose2d> blueReefPoses =
        new ArrayList<Pose2d>() {
          {
            // add(new Pose2d(2.890, 4.025, new Rotation2d(Units.degreesToRadians(1.00))));
            // add(new Pose2d(3.689, 2.642, new Rotation2d(Units.degreesToRadians(61.0))));
            // add(new Pose2d(5.285, 2.642, new Rotation2d(Units.degreesToRadians(121.0))));
            // add(new Pose2d(6.087, 4.025, new Rotation2d(Units.degreesToRadians(181.0))));
            // add(new Pose2d(5.285, 5.408, new Rotation2d(Units.degreesToRadians(241.0))));
            // add(new Pose2d(3.689, 5.408, new Rotation2d(Units.degreesToRadians(301.0))));

            add(new Pose2d(3.174, 4.025, new Rotation2d(Units.degreesToRadians(1.00)))); // AB
            add(new Pose2d(3.829, 2.902, new Rotation2d(Units.degreesToRadians(61.0)))); // CD
            add(new Pose2d(5.157, 2.902, new Rotation2d(Units.degreesToRadians(121.0)))); // EF
            add(new Pose2d(5.772, 4.025, new Rotation2d(Units.degreesToRadians(181.0)))); // GH
            add(new Pose2d(5.139, 5.144, new Rotation2d(Units.degreesToRadians(241.0)))); // IJ
            add(new Pose2d(3.849, 5.144, new Rotation2d(Units.degreesToRadians(301.0)))); // KL
          }
        };

    public static final List<Pose2d> redReefPoses =
        new ArrayList<Pose2d>() {
          {
            // add(new Pose2d(11.466, 4.025, new Rotation2d(Units.degreesToRadians(1.0))));
            // add(new Pose2d(12.265, 2.642, new Rotation2d(Units.degreesToRadians(61.0))));
            // add(new Pose2d(13.861, 2.642, new Rotation2d(Units.degreesToRadians(121.0))));
            // add(new Pose2d(14.663, 4.025, new Rotation2d(Units.degreesToRadians(181.0))));
            // add(new Pose2d(13.861, 5.408, new Rotation2d(Units.degreesToRadians(241.0))));
            // add(new Pose2d(12.265, 5.408, new Rotation2d(Units.degreesToRadians(301.0))));

            // add(new Pose2d(11.182, 4.025, new Rotation2d(Units.degreesToRadians(1.0))));
            // add(new Pose2d(12.125, 2.902, new Rotation2d(Units.degreesToRadians(61.0))));
            // add(new Pose2d(13.989, 2.902, new Rotation2d(Units.degreesToRadians(121.0))));
            // add(new Pose2d(14.978, 4.025, new Rotation2d(Units.degreesToRadians(181.0))));
            // add(new Pose2d(14.007, 5.144, new Rotation2d(Units.degreesToRadians(241.0))));
            // add(new Pose2d(12.105, 5.144, new Rotation2d(Units.degreesToRadians(301.0))));

            add(new Pose2d(11.750, 4.025, new Rotation2d(Units.degreesToRadians(1.0)))); // GH
            add(new Pose2d(12.405, 2.902, new Rotation2d(Units.degreesToRadians(61.0)))); // IJ
            add(new Pose2d(13.733, 2.902, new Rotation2d(Units.degreesToRadians(121.0)))); // KL
            add(new Pose2d(14.348, 4.025, new Rotation2d(Units.degreesToRadians(181.0)))); // AB
            add(new Pose2d(13.715, 5.144, new Rotation2d(Units.degreesToRadians(241.0)))); // CD
            add(new Pose2d(12.425, 5.144, new Rotation2d(Units.degreesToRadians(301.0)))); // EF
          }
        };

    public static final double leftOffset = 0.155;
    // public static final double L2ScoringOffset = 0.285;
    // public static final double L3ScoringOffset = 0.155;
    // public static final double L4ScoringOffset = 0.27;
    // public static final double topAlgaeScoringOffset = 0.23;
    // public static final double bottomAlgaeScoringOffset = 0.25;

    public static final List<Pose2d> blueSourcePoses =
        new ArrayList<Pose2d>() {
          {
            add(
                new Pose2d(
                    1.277, 7.096, new Rotation2d(Units.degreesToRadians(-53)))); // left  //307
            add(
                new Pose2d(
                    1.175, 0.938, new Rotation2d(Units.degreesToRadians(53)))); // right   //53
          }
        };
    public static final List<Pose2d> redSourcePoses =
        new ArrayList<Pose2d>() {
          {
            add(
                new Pose2d(
                    16.273, 7.096, new Rotation2d(Units.degreesToRadians(-127)))); // right //233
            add(
                new Pose2d(
                    16.375, 0.938, new Rotation2d(Units.degreesToRadians(127)))); // left  //127
          }
        };

    public static final List<Pose2d> blueBargePoses =
        new ArrayList<Pose2d>() {
          {
            add(new Pose2d(8.187, 5, new Rotation2d(Units.degreesToRadians(181))));
            add(new Pose2d(8.187, 6.5, new Rotation2d(Units.degreesToRadians(181))));
          }
        };
    public static final List<Pose2d> redBargePoses =
        new ArrayList<Pose2d>() {
          {
            add(new Pose2d(9.363, 3, new Rotation2d(Units.degreesToRadians(1))));
            add(new Pose2d(9.363, 1.5, new Rotation2d(Units.degreesToRadians(1))));
          }
        };
  }
}
