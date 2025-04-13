package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import frc.robot.Constants.CANDLE;
import frc.robot.subsystems.lights.LightSystem.Color;
import frc.robot.subsystems.lights.LightSystem.Segment;

public class LightSystemIOCANdle implements LightSystemIO {
  private final CANdle candle;
  private final CANdleConfiguration config = new CANdleConfiguration();

  public LightSystemIOCANdle() {
    candle = new CANdle(CANDLE.CANID, CANDLE.CANBUS);

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

  @Override
  public void updateInputs(LightSystemIOInputs inputs) {
    inputs.connected = true;
    inputs.current = candle.getCurrent();
  }

  @Override
  public void setBrightness(double percent) {
    candle.configBrightnessScalar(percent, 100);
  }

  @Override
  public void clearAnimation(Segment segment) {
    candle.clearAnimation(segment.animationSlot);
  }

  @Override
  public void setAnimation(Animation animation, Segment segment) {
    candle.animate(animation, segment.animationSlot);
  }

  @Override
  public void setColor(Color color, Segment segment) {
    clearAnimation(segment);
    candle.setLEDs(
        color.red, color.green, color.blue, color.white, segment.startIndex, segment.segmentSize);
  }
}
