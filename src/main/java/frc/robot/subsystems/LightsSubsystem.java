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
import frc.robot.Constants.CANDLE;
import frc.robot.Constants.LEDColor;

import org.littletonrobotics.junction.Logger;

public class LightsSubsystem extends SubsystemBase {
  private static final CANdle candle = new CANdle(CANDLE.CANID, CANDLE.CANBUS);
  private final CANdleConfiguration config = new CANdleConfiguration();

  private double m_current;

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

  public void setBrightness(double percent) {
    candle.configBrightnessScalar(percent, 100);
  }

  public static void setColor(Color color, Segment segment) {
    clearAnimation(segment);
    candle.setLEDs(
        color.red, color.green, color.blue, color.white, segment.startIndex, segment.segmentSize);
  }

  private static void setAnimation(Animation animation, Segment segment) {
    candle.animate(animation, segment.animationSlot);
  }

  public static void setFlowAnimation(Color color, double speed, Segment segment) {
    setAnimation(
        new ColorFlowAnimation(
            color.red,
            color.green,
            color.blue,
            color.white,
            speed,
            segment.segmentSize,
            Direction.Forward,
            segment.startIndex),
        segment);
  }

  public static void setFadeAnimation(Color color, double speed, Segment segment) {
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

  public static void setBandAnimation(Color color, double speed, Segment segment) {
    setAnimation(
        new LarsonAnimation(
            color.red,
            color.green,
            color.blue,
            color.white,
            speed,
            segment.segmentSize,
            BounceMode.Front,
            1,
            segment.startIndex),
        segment);
  }

  public static void setStrobeAnimation(Color color, double speed, Segment segment) {
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

  public static void setRainbowAnimation(double speed, Segment segment) {
    setAnimation(
        new RainbowAnimation(1, speed, segment.segmentSize, false, segment.startIndex), segment);
  }

  public static void clearAnimation(Segment segment) {
    candle.clearAnimation(segment.animationSlot);
  }

  public static void disableLEDs(Segment segment) {
    setColor(LEDColor.BLACK, segment);
  }

  public static class Segment {
    private int startIndex;
    public int segmentSize;
    public int animationSlot;

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
    m_current = candle.getCurrent();
    Logger.recordOutput("LED/current", m_current);
  }

  public double getCurrent() {
    return m_current;
  }
}
