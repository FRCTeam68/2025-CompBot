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

  // public static final class INTAKE {
  //   public static final int CANID = 20;
  //   public static final String CANBUS = "DRIVEbus";
  //   public static final double TAKE_NOTE_SPEED = 20;
  //   public static final double SPIT_NOTE_SPEED = 30;
  //   public static final double BUMP_VALUE = 0.2; // 50 counts / second = 10rps
  // }

  public static final class INTAKE_SHOOTER {
    public static final int CANID = 30;
    // public static final int RIGHT_CANID = 31;
    public static final String CANBUS = "rio";
    public static final double MAX_SPEED = 100; // rps
    public static final double CORAL_INTAKE_SPEED = 30;
    public static final double CORAL_SHOOT_SPEED = -20;
    public static final double CORAL_SHOOT_TIMEOUT = 2;
    public static final double BUMP_VALUE = 1; // rotations
  }

  public static final class ELEVATOR { // old AngleSubSystem
    public static final int LEFT_CANID = 32;
    public static final int RIGHT_CANID = 33;
    public static final String CANBUS = "rio";

    // increasing value makes shooter side go down
    // decreasing value makes shooter side go up
    // as of Feb 22nd we cannot go steeper than speaker position of 0
    // so AMP and TRAP are also 0
    // motor rotations
    public static final double MIN_POSITION = 0;
    public static final double MAX_POSITION = 50;
    public static final double L4 = 30;
    public static final double L3 = 25;
    public static final double L2 = 15;
    public static final double L1 = 5;
    public static final double A2 = 10;
    public static final double A1 = 2;
    public static final double BUMP_VALUE = .5; // rotations
  }

  public static final class WRIST {
    public static final int CANID = 31; // old shooterSubSystem right, top
    public static final double MIN_POSITION = 0;
    public static final double MAX_POSITION = 10;
    public static final double L4 = 22;
    public static final double L3 = 15;
    public static final double L2 = 15;
    public static final double L1 = 5;
    public static final double A2 = 20;
    public static final double A1 = 20;
    public static final double BUMP_VALUE = .5; // rotations
  }

  public static final class CLIMBER {
    public static final int LEFT_CANID = 40;
    public static final int RIGHT_CANID = 41;
    public static final String CANBUS = "DRIVEbus";
    public static final double MAX_HEIGHT =
        132; // 122;  // old hooks: 115;   //100 rotates is about 9in
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
