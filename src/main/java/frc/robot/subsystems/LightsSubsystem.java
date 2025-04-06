package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDLE;
import org.littletonrobotics.junction.Logger;

public class LightsSubsystem extends SubsystemBase {
  private static final CANdle candle = new CANdle(CANDLE.CANID, CANDLE.CANBUS);
  private final CANdleConfiguration config = new CANdleConfiguration();

  // Team colors
  public static final Color orange = new Color(255, 25, 0);
  public static final Color black = new Color(0, 0, 0);

  // Game piece colors
  public static final Color yellow = new Color(242, 60, 0);
  public static final Color purple = new Color(184, 0, 185);

  // Indicator colors
  public static final Color white = new Color(255, 230, 220);
  public static final Color green = new Color(56, 209, 0);
  public static final Color blue = new Color(0, 0, 255);
  public static final Color red = new Color(255, 0, 0);

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
    candle.clearAnimation(0);
    candle.clearAnimation(1);
    candle.clearAnimation(2);
    candle.clearAnimation(3);
    candle.clearAnimation(4);
  }

  public void setBrightness(double percent) {
    candle.configBrightnessScalar(percent, 100);
  }

  public Command clearSegmentCommand(LEDSegment segment) {
    return runOnce(
        () -> {
          segment.clearAnimation();
          segment.disableLEDs();
        });
  }

  public static enum LEDSegment {
    LED0(0, 1, 0), // camera - hopper
    LED1(1, 1, 0), // camera - reef
    LED2(2, 1, 0), // CANrange - intake
    LED3(3, 1, 0), // CANrange - reef post
    LED4(4, 1, 0), // rio can bus
    LED5(5, 1, 0), // CAnivore can bus
    LED6(6, 1, 0),
    LED7(7, 1, 0),

    // auton setup
    autonYLeft(45, 4, 0),
    autonRLeft(49, 3, 0),
    autonXLeft(52, 2, 0),
    autonXRight(54, 2, 0),
    autonRRight(56, 3, 0),
    autonYRight(59, 4, 0),

    // there is 1 long strand:  36 + 18 + 36, so 90 LEDs total
    leftside(8, 37, 1),
    middle(45, 18, 2),
    rightside(63, 36, 3),
    all(8, 92, 4);

    public final int startIndex;
    public final int segmentSize;
    public final int animationSlot;

    private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
      this.startIndex = startIndex;
      this.segmentSize = segmentSize;
      this.animationSlot = animationSlot;
    }

    public void setColor(Color color) {
      clearAnimation();
      candle.setLEDs(color.red, color.green, color.blue, 0, startIndex, segmentSize);
    }

    private void setAnimation(Animation animation) {
      candle.animate(animation, animationSlot);
    }

    public void fullClear() {
      clearAnimation();
      disableLEDs();
    }

    public void clearAnimation() {
      candle.clearAnimation(animationSlot);
    }

    public void disableLEDs() {
      setColor(black);
    }

    public void setFlowAnimation(Color color, double speed) {
      setAnimation(
          new ColorFlowAnimation(
              color.red,
              color.green,
              color.blue,
              0,
              speed,
              segmentSize,
              Direction.Forward,
              startIndex));
    }

    public void setFadeAnimation(Color color, double speed) {
      setAnimation(
          new SingleFadeAnimation(
              color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
    }

    public void setBandAnimation(Color color, double speed) {
      setAnimation(
          new LarsonAnimation(
              color.red,
              color.green,
              color.blue,
              0,
              speed,
              segmentSize,
              BounceMode.Front,
              1,
              startIndex));
    }

    public void setStrobeAnimation(Color color, double speed) {
      setAnimation(
          new StrobeAnimation(
              color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
    }

    public void setRainbowAnimation(double speed) {
      setAnimation(new RainbowAnimation(1, speed, segmentSize, false, startIndex));
    }
  }

  public static class Color {
    public int red;
    public int green;
    public int blue;

    public Color(int red, int green, int blue) {
      this.red = red;
      this.green = green;
      this.blue = blue;
    }

    /**
     * Highly imperfect way of dimming the LEDs. It does not maintain color or accurately adjust
     * perceived brightness.
     *
     * @param dimFactor
     * @return The dimmed color
     */
    public Color dim(double dimFactor) {
      int newRed = (int) (MathUtil.clamp(red * dimFactor, 0, 200));
      int newGreen = (int) (MathUtil.clamp(green * dimFactor, 0, 200));
      int newBlue = (int) (MathUtil.clamp(blue * dimFactor, 0, 200));

      return new Color(newRed, newGreen, newBlue);
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

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("candle");
    builder.addDoubleProperty("candle current", this::getCurrent, null);
  }
}
