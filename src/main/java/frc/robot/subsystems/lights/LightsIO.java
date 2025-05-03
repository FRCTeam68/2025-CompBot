package frc.robot.subsystems.lights;

import com.ctre.phoenix6.controls.ControlRequest;
import org.littletonrobotics.junction.AutoLog;

public interface LightsIO {
  @AutoLog
  static class LightsIOInputs {
    public boolean connected = false;
    public double outputCurrent = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(LightsIOInputs inputs) {}

  /**
   * Set brightness for all LEDs
   *
   * @param percent Value from [0, 1] that will scale the LED brightness
   */
  default void setBrightness(double percent) {}

  /**
   * Clear animation of a segment
   *
   * @param segment LED segment to clear animation
   */
  default void clearAnimation(int animationSlot) {}

  /**
   * Apply animation
   *
   * @param animation The animation that CANdle will run. If this is null, it will clear the
   *     animation at the specified slot
   * @param segment LED segment to use animation slot
   */
  default void setControl(ControlRequest request) {}
}
