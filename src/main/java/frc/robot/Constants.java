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

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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
    public static final double CORAL_INTAKE_SPEED = 30;
    public static final double CORAL_SHOOT_SPEED = 40;
    public static final double CORAL_SHOOT_TIMEOUT = 2;
    public static final double ALGAE_INTAKE_SPEED = -40;
    public static final double ALGAE_SHOOT_SPEED = 20;
    public static final double ALGAE_SHOOT_TIMEOUT = 2;
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
    public static final double MIN_POSITION = 0;
    public static final double MIN_POSITION_TO_CLEAR_ELEVATOR = 15;
    public static final double MAX_POSITION_AT_ELEVATOR_MIN = 28;
    public static final double MAX_POSITION_AT_P1 = 34;
    public static final double L4 = 4.5;
    public static final double L3 = 4.5;
    public static final double L2 = 4.5;
    public static final double L1 = 0;
    public static final double CRADLE = 15;
    public static final double A2 = 27.5;
    public static final double A1 = 27.5;
    public static final double P1 = 34;
    public static final double PRENET = 21;
    public static final double SHOOTNET = 6;
    public static final double BUMP_VALUE = .5; // rotations
    public static final Slot0Configs SLOT0_CONFIGS =
        new Slot0Configs().withKP(6).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    public static final MotionMagicConfigs MOTIONMAGIC_CONFIGS =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(20)
            .withMotionMagicAcceleration(40)
            .withMotionMagicJerk(400);
  }

  public static final class ELEVATOR { // old AngleSubSystem
    public static final int LEFT_CANID = 32;
    public static final int RIGHT_CANID = 33;
    public static final String CANBUS = "rio";
    public static final double MIN_POSITION = 0;
    public static final double MIN_POSITION_AT_P1 = 3.4;
    public static final double MAX_POSITION = 26.5;
    public static final double MAX_POSITION_WRIST_NOT_CLEAR = 1.5;
    public static final double L4 = 22;
    public static final double L3 = 11.4;
    public static final double L2 = 4;
    public static final double L1 = 0;
    public static final double A2 = 13.5;
    public static final double A1 = 7;
    public static final double P1 = 3.4;
    public static final double PRENET = 20;
    public static final double SHOOTNET = 26;
    public static final double BUMP_VALUE = .5; // rotations
    public static final Slot0Configs SLOT0_CONFIGS =
        new Slot0Configs().withKP(4.8).withKI(0).withKD(0).withKS(0.25).withKV(0.1).withKA(0);
    public static final MotionMagicConfigs MOTIONMAGIC_CONFIGS =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(20)
            .withMotionMagicAcceleration(40)
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
    public static final double GOAL = 220;
    public static final double MAX_HEIGHT = 300;
    public static final Slot0Configs SLOT0_CONFIGS =
        new Slot0Configs().withKP(.1).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    public static final MotionMagicConfigs MOTIONMAGIC_CONFIGS =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(20)
            .withMotionMagicAcceleration(40)
            .withMotionMagicJerk(400);
  }

  public static final class RED_TAGS {
    public static final int[] stage = {11, 12, 13};
  }

  public static final class BLUE_TAGS {
    public static final int[] stage = {14, 15, 16};
  }

  public static final class LightsConstants {
    public static final int CANDLE_PORT = 60;
  }
}
