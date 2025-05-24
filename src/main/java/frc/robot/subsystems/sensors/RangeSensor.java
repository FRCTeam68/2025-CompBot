package frc.robot.subsystems.sensors;

import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDColor;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.Segment;
import org.littletonrobotics.junction.Logger;

public class RangeSensor extends SubsystemBase {
  private final Lights LED;
  private final RangeSensorIO io;
  private final String name;
  protected final RangeSystemIOInputsAutoLogged inputs = new RangeSystemIOInputsAutoLogged();

  private final Segment indicator;
  private RGBWColor indicatorColor = LEDColor.RED;
  private RGBWColor prevIndicatorColor = new RGBWColor();

  private final Alert disconnectedAlert;

  public RangeSensor(Lights LED, String name, RangeSensorIO io, Segment... indicator) {
    this.LED = LED;
    this.io = io;
    this.name = name;
    this.indicator = indicator[0];

    disconnectedAlert = new Alert(name + " sensor disconnected.", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Sensors/" + name, inputs);
    disconnectedAlert.set(!inputs.connected);

    if (indicator != null) {
      // determine indicator color
      if (inputs.detected) {
        indicatorColor = LEDColor.BLUE;
      } else if (inputs.connected) {
        indicatorColor = LEDColor.GREEN;
      } else {
        indicatorColor = LEDColor.RED;
      }

      if (indicatorColor != prevIndicatorColor) {
        LED.setSolidColor(
            indicatorColor.scaleBrightness(Lights.getOnboardLEDBrightness()), indicator);
      }
    }
  }

  public boolean isDetected() {
    return inputs.detected;
  }

  public double getDistance_mm() {
    return inputs.distance_mm;
  }
}
