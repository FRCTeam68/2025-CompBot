package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
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
import frc.robot.Constants.LightsConstants;
import org.littletonrobotics.junction.Logger;

public class LightsSubsystem extends SubsystemBase {
  private static final CANdle candle = new CANdle(LightsConstants.CANDLE_PORT);

  // Team colors
  public static final Color orange = new Color(255, 25, 0);
  public static final Color black = new Color(0, 0, 0);

  // Game piece colors
  public static final Color yellow = new Color(242, 60, 0);
  public static final Color purple = new Color(184, 0, 185);

  // Indicator colors
  public static final Color white = new Color(255, 230, 220);
  public static final Color green = new Color(56, 209, 0);
  // public static final Color blue = new Color(8, 32, 255);
  public static final Color blue = new Color(0, 0, 255);
  public static final Color red = new Color(255, 0, 0);

  private double m_current;

  public LightsSubsystem() {
    candle.configFactoryDefault();
    candle.configVBatOutput(VBatOutputMode.On);
    candle.configLEDType(LEDStripType.RGB);
    candle.clearAnimation(0);
    candle.clearAnimation(1);
    candle.clearAnimation(2);
    candle.clearAnimation(3);
    candle.clearAnimation(4);
    candle.configBrightnessScalar(1);

    // CANdleConfiguration candleConfiguration = new CANdleConfiguration();
    // // candleConfiguration.statusLedOffWhenActive = false; //true;
    // // candleConfiguration.disableWhenLOS = false;
    // candleConfiguration.stripType = LEDStripType.RGB;
    // candleConfiguration.brightnessScalar = 1.0;
    // // candleConfiguration.vBatOutputMode = VBatOutputMode.On; //VBatOutputMode.Modulated
    // candle.configAllSettings(candleConfiguration, 100);

    // setDefaultCommand(defaultCommand());
  }

  public void setBrightness(double percent) {
    candle.configBrightnessScalar(percent, 100);
  }

  // public Command defaultCommand() {
  //     return runOnce(() -> {
  //         // LEDSegment.BatteryIndicator.fullClear();
  //         // LEDSegment.PressureIndicator.fullClear();
  //         // LEDSegment.MastEncoderIndicator.fullClear();
  //         // LEDSegment.BoomEncoderIndicator.fullClear();
  //         // LEDSegment.WristEncoderIndicator.fullClear();
  //         LEDSegment.Do1to4.setColor(yellow);
  //         LEDSegment.Do5to8.setColor(yellow);
  //         LEDSegment.side1.setColor(orange);
  //         LEDSegment.side1heading.setColor(orange);
  //         LEDSegment.side1distance.setColor(orange);
  //         LEDSegment.side1target.setColor(orange);
  //         // LEDSegment.side2.setColor(blue);
  //         // LEDSegment.side3.setColor(orange);
  //         // LEDSegment.side4.setColor(blue);
  //     });
  // }

  public Command clearSegmentCommand(LEDSegment segment) {
    return runOnce(
        () -> {
          segment.clearAnimation();
          segment.disableLEDs();
        });
  }

  public static enum LEDSegment {
    // BatteryIndicator(0, 2, 0),
    // PressureIndicator(2, 2, 1),
    // MastEncoderIndicator(4, 1, -1),
    // BoomEncoderIndicator(5, 1, -1),
    // WristEncoderIndicator(6, 1, -1),
    // DriverStationIndicator(7, 1, -1),
    Do1to4(0, 4, 0),
    Do5to8(4, 4, 1),
    // side1(8, 7, 2),
    // side1heading(15,1,3),
    // side1distance(16,1,4),
    // side1target(17,1,-1),
    // side2(18, 7, 2),
    // side3(28, 7, 2),
    // side4(38, 7, 2);

    // side1(8, 4, 2),
    // side1heading(12,1,3),
    // side1distance(13,1,4),
    // side1target(14,1,-1),
    // side2(15, 4, 5),
    // side3(22, 4, 6),
    // side4(29, 4, 7);

    // there are 4 segments of 7, so 28 LEDs
    side1(8, 25, 2),
    side1target(33, 1, 3),
    side1distance(34, 1, 4),
    side1heading(35, 1, -1);
    // side2(36, 28, 5);

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
