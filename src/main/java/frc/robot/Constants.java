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

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = true;
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
        new Pose2d(Meters.of(7.26), Meters.of(2.65), new Rotation2d(180));
    public static final Pose2d middle =
        new Pose2d(right.getMeasureX(), Meters.of(4.16), right.getRotation());
    public static final Pose2d left =
        new Pose2d(right.getMeasureX(), Meters.of(5.60), right.getRotation());
  }

  public static final class LED {
    public static final int PWMPORT = 0;
    public static final int BUFFERSIZE = 120;
  }

  public static final class ROLLER {
    public static final double MAX_SPEED = 100;
    public static final double MAX_VOLTAGE = 12;
  }

  public static final class INTAKE_SHOOTER {
    public static final int CANID = 30;
    // public static final int RIGHT_CANID = 31;
    public static final String CANBUS = "rio";
    public static final double MAX_SPEED = 100; // rps
    // coral
    public static final double CORAL_INTAKE_SPEED = 30;
    public static final double CORAL_INTAKE_INDEX_SPEED = 7; // speed to find back edge
    public static final double CORAL_INTAKE_INDEX_REVERSE = 3; // revolutions to reverse
    public static final double CORAL_SHOOT_SPEED = 30; // 40
    public static final double CORAL_L1_SHOOT_SPEED = -10; // 40
    public static final double CORAL_SHOOT_TIMEOUT = 1;
    // algae
    public static final double ALGAE_INTAKE_SPEED = -40;
    public static final double ALGAE_HOLD_SPEED = -5;
    public static final double ALGAE_SHOOT_SPEED = 20;
    public static final double ALGAE_NET_SHOOT_SPEED = 80;
    public static final double ALGAE_NET_SHOOT_DELAY = .13;
    public static final double ALGAE_SHOOT_TIMEOUT = 1;
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
    // LaserCAN distance sensor
    public static final int CANID = 37;
    public static final String CANBUS = "rio";
    public static final double THRESHOLD = 20; // mm
  }

  public static final class WRIST {
    public static final int CANID = 31; // old shooterSubSystem right, top
    public static final int CANCODER_CANID = 36;
    public static final String CANBUS = "rio";
    public static String POSITION_SCORING_ELEMENT = "NULL";
    public static final double MIN_POSITION = 0;
    public static final double MIN_SLOT1_TO_ELEVATE = 4.1; // 3.8;
    public static final double SLOT1_TO_ELEVATE = 4.2; // packaged position to lift elevator
    public static final double MAX_SLOT1_TO_ELEVATE = 4.3; // 4.0;
    public static final double MIN_POSITION_TO_CLEAR_ELEVATOR = 14.5; // min slot2
    public static final double MAX_POSITION_AT_ELEVATOR_MIN = 26; // max slot2
    public static final double MAX_POSITION_AT_P1 = 34;
    public static final double MAX_POSITION = 34;
    public static final double INTAKE = 1.5;
    public static final double L1 = 17;
    public static double L1_OFFSET = 0;
    public static final double L2 = 4.2; // 3.9; // DO NOT MODIFY WITHOUT CHANGING
    public static final double L3 = 4.2; // 3.9; // SEQUENCING LOGIC
    public static final double L4 = 4.2; // 3.9; // all 3 of these must be the same value
    public static final double CRADLE = 12;
    public static final double SHOOTNET = 5;
    public static final double PRENET = 22;
    public static final double A2 = 27.5;
    public static final double A1 = 27.5;
    public static final double P1 = 33;
    public static final double BUMP_VALUE = .5; // rotations
    public static final double SAFE = 14.5; // minimum position to move full elevator travel
    public static final double ERROR = 0.5;
    public static final double CANCODER_OFFSET = 0.064453125;
    public static final double CANCODER_FACTOR = 1.4634 / 0.02197;
    public static final Slot0Configs SLOT0_CONFIGS =
        new Slot0Configs().withKP(40).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    public static final MotionMagicConfigs MOTIONMAGIC_CONFIGS =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(60) // 60  5
            .withMotionMagicAcceleration(240) // 240  15
            .withMotionMagicJerk(10000);
  }

  public static final class ELEVATOR { // old AngleSubSystem
    public static final int LEFT_CANID = 32;
    public static final int RIGHT_CANID = 33;
    public static final String CANBUS = "rio";
    public static final double MAX_POSITION = 27; // 26.5; // MAX_BLOCK6
    public static final double MAX_POSITION_BLOCK5 = 24; // ????
    public static final double MAX_POSITION_BLOCK4 = 21; // ????
    public static final double SAFE_IN_BLOCK4 = 12.5;
    public static final double MIN_POSITION_BLOCK4 = 10.5; // ????
    public static final double MAX_POSITION_BLOCK2 = 5.1; // ????
    public static final double MIN_POSITION_AT_P1 = 3.4; // MIN_BLOCK2
    public static final double MAX_POSITION_BLOCK0 = 1;
    public static final double MIN_POSITION = 0; // /MIN_BLOCK0
    public static final double SHOOTNET = 26.5;
    public static final double L4 = 23.5;
    public static final double PRENET = 26.5;
    public static final double A2 = 15.5;
    public static final double L3 = 12;
    public static final double A1 = 9;
    public static final double L2 = 4.5;
    public static final double P1 = 3.4;
    public static final double L1 = 8;
    public static double L1_OFFSET = 0;
    public static final double INTAKE = 0;
    public static final double BUMP_VALUE = .5; // rotations
    // sequencing contants
    public static final double MAX_LOW_SAFE = 1;
    public static final double MIN_MID_SAFE = 11.78;
    public static final double MAX_MID_SAFE = 16;
    public static final double MIN_HIGH_SAFE = 24;
    public static final double MAX_LOW_WRIST_MOVE_FROM_SAFE =
        3; // height to start wrist move if wrist is in safe position
    //
    public static final Slot0Configs SLOT0_CONFIGS =
        new Slot0Configs().withKP(10).withKI(0).withKD(0).withKS(0.5).withKV(0.2).withKA(0);
    public static final MotionMagicConfigs MOTIONMAGIC_CONFIGS =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(40) // 40  5
            .withMotionMagicAcceleration(120) // 120  60
            .withMotionMagicJerk(400);
  }

  public static final class ELEVATOR_SENSOR {
    // distance sensor
    public static final int CANID = 47;
    public static final String CANBUS = "rio";
    public static final double THRESHOLD = 50; // mm
  }

  public static final class CLIMBER {
    public static final int CANID = 40;
    public static final String CANBUS = "rio";
    public static final double GOAL = 102; // 52;
    public static final double MAX_HEIGHT = 60;
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

  // public static final class RED_TAGS {
  //   public static final int[] stage = {11, 12, 13};
  // }

  // public static final class BLUE_TAGS {
  //   public static final int[] stage = {14, 15, 16};
  // }

  public static final class LightsConstants {
    public static final int CANDLE_PORT = 60;
  }
}
