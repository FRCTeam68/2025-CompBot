package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANDLE;
import frc.robot.Constants.LEDColor;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class LightsSubsystem extends SubsystemBase {
  private static final CANdle candle = new CANdle(CANDLE.CANID, CANDLE.CANBUS);
  private final CANdleConfiguration config = new CANdleConfiguration();

  private double m_current = 0;
  private static double defaultAnimationSpeed = 4;

  public LightsSubsystem() {
    config.brightnessScalar = CANDLE.BRIGHTNESS_SCALAR;
    config.disableWhenLOS = CANDLE.DISABLE_WHEN_LOS;
    config.statusLedOffWhenActive = CANDLE.STATUS_OFF_WHEN_ACTIVE;
    config.stripType = CANDLE.LED_STRIP_TYPE;
    config.v5Enabled = CANDLE.V5_ENABLED;
    config.vBatOutputMode = CANDLE.V_BAT_OUTPUT_MODE;

    candle.configAllSettings(config, 100);

    // clear animation slots
    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
      candle.clearAnimation(i);
    }
  }

  /**
   * Set brightness for all LEDs
   *
   * @param percent Value from [0, 1] that will scale the LED brightness
   */
  public void setBrightness(double percent) {
    candle.configBrightnessScalar(percent, 100);
  }

  /**
   * Set default animation speed. Used for all animations if speed is not specified
   *
   * @param speed How fast should the color travel the strip [0, 1]
   */
  public void setDefaultAnimationSpeed(double speed) {
    defaultAnimationSpeed = speed;
  }

  /**
   * Clear animation of a segment
   *
   * @param segment LED segment to clear animation
   */
  public static void clearAnimation(Segment segment) {
    candle.clearAnimation(segment.animationSlot);
  }

  /**
   * Apply animation
   *
   * @param animation The animation that CANdle will run. If this is null, it will clear the
   *     animation at the specified slot
   * @param segment LED segment to use animation slot
   */
  private static void setAnimation(Animation animation, Segment segment) {
    candle.animate(animation, segment.animationSlot);
  }

  /**
   * Turn off LEDs of a segment
   *
   * @param segment LED segment to turn off
   */
  public static void disableLEDs(Segment segment) {
    setColor(LEDColor.BLACK, segment);
  }

  /**
   * Set static color for LED segment
   *
   * @param color Color of the LED
   * @param segment LED segment to apply color change
   */
  public static void setColor(Color color, Segment segment) {
    clearAnimation(segment);
    candle.setLEDs(
        color.red, color.green, color.blue, color.white, segment.startIndex, segment.segmentSize);
  }

  /**
   * Set flowing animation for LED segment
   *
   * <p>using default values
   *
   * @param color Color of the LED
   * @param segment LED segment to apply animation
   */
  public static void setFlowAnimation(Color color, Segment segment) {
    setFlowAnimation(color, segment, defaultAnimationSpeed, Direction.Forward);
  }

  /**
   * Set flowing animation for LED segment
   *
   * @param color Color of the LED
   * @param segment LED segment to apply animation
   * @param speed How fast should the color travel the strip [0, 1]
   * @param direction What direction should the color move in
   */
  public static void setFlowAnimation(
      Color color, Segment segment, double speed, Direction direction) {
    setAnimation(
        new ColorFlowAnimation(
            color.red,
            color.green,
            color.blue,
            color.white,
            speed,
            segment.segmentSize,
            direction,
            segment.startIndex),
        segment);
  }

  /**
   * Set fading animation for LED segment
   *
   * <p>using default values
   *
   * @param color Color of the LED
   * @param segment LED segment to apply animation
   */
  public static void setFadeAnimation(Color color, Segment segment) {
    setFadeAnimation(color, segment, defaultAnimationSpeed);
  }

  /**
   * Set fading animation for LED segment
   *
   * @param color Color of the LED
   * @param segment LED segment to apply animation
   * @param speed How fast should the color travel the strip [0, 1]
   */
  public static void setFadeAnimation(Color color, Segment segment, double speed) {
    setAnimation(
        new SingleFadeAnimation(
            color.red,
            color.green,
            color.blue,
            color.white,
            speed,
            segment.segmentSize,
            segment.startIndex),
        segment);
  }

  /**
   * Set banding animation for LED segment
   *
   * <p>using default values
   *
   * @param color Color of the LED
   * @param segment LED segment to apply animation
   */
  public static void setBandAnimation(Color color, Segment segment) {
    setBandAnimation(color, segment, defaultAnimationSpeed, BounceMode.Front, 1);
  }

  /**
   * Set banding animation for LED segment
   *
   * @param color Color of the LED
   * @param segment LED segment to apply animation
   * @param speed How fast should the color travel the strip [0, 1]
   * @param mode How the pocket of LEDs will behave once it reaches the end of the strip
   * @param size How large the pocket of LEDs are [0, 7]
   */
  public static void setBandAnimation(
      Color color, Segment segment, double speed, BounceMode mode, int size) {
    setAnimation(
        new LarsonAnimation(
            color.red,
            color.green,
            color.blue,
            color.white,
            speed,
            segment.segmentSize,
            mode,
            size,
            segment.startIndex),
        segment);
  }

  /**
   * Set strobing animation for LED segment
   *
   * <p>using default values
   *
   * @param color Color of the LED
   * @param segment LED segment to apply animation
   */
  public static void setStrobeAnimation(Color color, Segment segment) {
    setStrobeAnimation(color, segment, defaultAnimationSpeed);
  }

  /**
   * Set strobing animation for LED segment
   *
   * @param color Color of the LED
   * @param segment LED segment to apply animation
   * @param speed How fast should the color travel the strip [0, 1]
   */
  public static void setStrobeAnimation(Color color, Segment segment, double speed) {
    setAnimation(
        new StrobeAnimation(
            color.red,
            color.green,
            color.blue,
            color.white,
            speed,
            segment.segmentSize,
            segment.startIndex),
        segment);
  }

  /**
   * Set rainbow animation for LED segment
   *
   * <p>using default values
   *
   * @param color Color of the LED
   * @param segment LED segment to apply animation
   */
  public static void setRainbowAnimation(Segment segment) {
    setRainbowAnimation(segment, defaultAnimationSpeed, false);
  }

  /**
   * Set rainbow animation for LED segment
   *
   * @param color Color of the LED
   * @param segment LED segment to apply animation
   * @param speed How fast should the color travel the strip [0, 1]
   * @param reverseDirection True to reverse the animation direction, so instead of going "toward"
   *     the CANdle, it will go "away" from the CANdle.
   */
  public static void setRainbowAnimation(Segment segment, double speed, boolean reverseDirection) {
    setAnimation(
        new RainbowAnimation(1, speed, segment.segmentSize, reverseDirection, segment.startIndex),
        segment);
  }

  public static class Segment {
    private int startIndex;
    public int segmentSize;
    public int animationSlot;

    /**
     * LED segment
     *
     * @param startIndex Where to start the LED segment
     * @param segmentSize Number of LEds in the segment
     * @param animationSlot The animation slot to use for the animation, range is [0,
     *     getMaxSimultaneousAnimationCount()] exclusive
     */
    public Segment(int startIndex, int segmentSize, int animationSlot) {
      this.startIndex = startIndex;
      this.segmentSize = segmentSize;
      this.animationSlot = animationSlot;
    }
  }

  public static class Color {
    public int red;
    public int green;
    public int blue;
    public int white;

    /**
     * LED color
     *
     * @param red The amount of Red to set, range is [0, 255]
     * @param green The amount of Green to set, range is [0, 255]
     * @param blue The amount of Blue to set, range is [0, 255]
     * @param white The amount of White to set, range is [0, 255]. This only applies for LED strips
     *     with white in them
     */
    public Color(int red, int green, int blue, int white) {
      this.red = red;
      this.green = green;
      this.blue = blue;
      this.white = white;
    }

    /**
     * Highly imperfect way of dimming the LEDs. It does not maintain color or accurately adjust
     * perceived brightness.
     *
     * @param dimFactor
     * @return The dimmed color
     */
    public Color dim(double dimFactor) {
      int newRed = (int) (MathUtil.clamp(red * dimFactor, 0, 255));
      int newGreen = (int) (MathUtil.clamp(green * dimFactor, 0, 255));
      int newBlue = (int) (MathUtil.clamp(blue * dimFactor, 0, 255));
      int newWhite = (int) (MathUtil.clamp(white * dimFactor, 0, 255));

      return new Color(newRed, newGreen, newBlue, newWhite);
    }
  }

  @Override
  public void periodic() {
    m_current = Constants.currentMode == Mode.REAL ? candle.getCurrent() : 0;
    Logger.recordOutput("LED/current", m_current);
  }
}
