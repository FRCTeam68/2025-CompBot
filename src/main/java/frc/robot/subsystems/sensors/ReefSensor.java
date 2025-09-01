package frc.robot.subsystems.sensors;

import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDColor;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.Segment;
import org.littletonrobotics.junction.Logger;

public class ReefSensor extends SubsystemBase {
  private final Lights LED;
  private final RangeSensorIO io;
  protected final RangeSystemIOInputsAutoLogged inputs = new RangeSystemIOInputsAutoLogged();

  private final Segment indicator = new Segment(4, 4, 0);
  private RGBWColor indicatorColor = LEDColor.RED;
  private RGBWColor prevIndicatorColor = new RGBWColor();

  private final Alert disconnectedAlert =
      new Alert("Reef CANrange sensor disconnected.", AlertType.kWarning);

  public ReefSensor(Lights LED, RangeSensorIO io) {
    this.LED = LED;
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Sensors/Reef CANrange", inputs);
    disconnectedAlert.set(!inputs.connected);

    // determine indicator color
    if (inputs.detected) {
      indicatorColor = LEDColor.BLUE;
    } else if (inputs.connected) {
      indicatorColor = LEDColor.GREEN;
    } else {
      indicatorColor = LEDColor.RED;
    }

    if (indicatorColor != prevIndicatorColor) {
      // only send when color changes so not using CAN bus every 20ms
      LED.setSolidColor(indicatorColor, indicator);
      prevIndicatorColor = indicatorColor;
    }
  }

  public boolean isDetected() {
    return inputs.detected;
  }

  public double getDistance_mm() {
    return inputs.distance_mm;
  }
}
