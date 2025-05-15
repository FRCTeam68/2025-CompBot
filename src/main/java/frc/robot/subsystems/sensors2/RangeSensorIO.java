package frc.robot.subsystems.sensors2;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import org.littletonrobotics.junction.AutoLog;

public interface RangeSensorIO {
  @AutoLog
  static class RangeSystemIOInputs {
    public boolean connected = false;
    public boolean detected = false;
    public double distance_mm = 0.0;
  }

  default void updateInputs(RangeSystemIOInputs inputs) {}

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
  public default void setPID(Slot0Configs config0, Slot1Configs config1) {}

  /** Set motion magic velocity, acceleration and jerk. */
  public default void setMotionMagic(MotionMagicConfigs newconfig) {}
}
