package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  static class ElevatorIOInputs {
    public boolean connected = false;
    public double positionRotations = 0.0;
    public double velocityRotsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public boolean followerConnected = false;
    public double followerAppliedVoltage = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerTorqueCurrentAmps = 0.0;
    public double followerTempCelsius = 0.0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  /* Run motor at volts */
  default void setVolts(double volts) {}

  /* Run mechanism at position */
  default void setPosition(double position, int slot) {}

  /* Stop motor */
  default void stop() {}

  default void zero() {}

  /**
   * Set slot configs
   *
   * <ul>
   *   <li><b>Available slots:</b> [0,2]
   * </ul>
   *
   * @param newconfig PID gains
   */
  public default void setPID(SlotConfigs... newconfig) {}

  /** Set motion magic velocity, acceleration and jerk. */
  public default void setMotionMagic(MotionMagicConfigs newconfig) {}
}
