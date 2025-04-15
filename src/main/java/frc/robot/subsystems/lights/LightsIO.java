package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.Animation;
import frc.robot.subsystems.lights.Lights.Color;
import frc.robot.subsystems.lights.Lights.Segment;
import org.littletonrobotics.junction.AutoLog;

public interface LightsIO {
  @AutoLog
  static class LightsIOInputs {
    public boolean connected = false;
    public double current = 0.0;
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
  default void clearAnimation(Segment segment) {}

  /**
   * Apply animation
   *
   * @param animation The animation that CANdle will run. If this is null, it will clear the
   *     animation at the specified slot
   * @param segment LED segment to use animation slot
   */
  default void setAnimation(Animation animation, Segment segment) {}

  /**
   * Set static color for LED segment
   *
   * @param color Color of the LED
   * @param segment LED segment to apply color change
   */
  default void setColor(Color color, Segment segment) {}
}
