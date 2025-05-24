// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import org.littletonrobotics.junction.AutoLog;

public interface RollerSystemIO {
  @AutoLog
  static class RollerSystemIOInputs {
    public boolean connected = false;
    public double positionRads = 0.0;
    public double positionRotations = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double velocityRotsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(RollerSystemIOInputs inputs) {}

  /* Run rollers at volts */
  default void setVolts(double volts) {}

  /* Run rollers at speed */
  default void setSpeed(double speed) {}

  /* Run rollers at position */
  default void setPosition(double rotations) {}

  /* Run rollers at position with feedforward */
  default void setPosition(double rotations, double feedforward, int slot) {}

  /* Stop rollers */
  default void stop() {}

  default void zero() {}

  /** Set P, I, and D gains for closed loop control on drive motor. */
  public default void setPID(SlotConfigs... newConfig) {}

  /** Set motion magic velocity, acceleration and jerk. */
  public default void setMotionMagic(MotionMagicConfigs newconfig) {}
}
