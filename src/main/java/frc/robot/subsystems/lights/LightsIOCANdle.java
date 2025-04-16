package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import frc.robot.subsystems.lights.Lights.Color;
import frc.robot.subsystems.lights.Lights.Segment;

public class LightsIOCANdle implements LightsIO {
  // Hardware
  private final CANdle candle = new CANdle(60, "rio");

  // Config
  private final CANdleConfiguration config = new CANdleConfiguration();

  public LightsIOCANdle() {
    config.brightnessScalar = 1;
    config.disableWhenLOS = false;
    config.statusLedOffWhenActive = false;
    config.stripType = LEDStripType.GRB;
    config.v5Enabled = true; // 5 volt output
    config.vBatOutputMode = VBatOutputMode.Off; // 12 volt output
    candle.configAllSettings(config, 100);

    // clear animation slots
    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
      candle.clearAnimation(i);
    }
  }

  @Override
  public void updateInputs(LightsIOInputs inputs) {
    inputs.connected = candle.getBusVoltage() != 0.0;
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
