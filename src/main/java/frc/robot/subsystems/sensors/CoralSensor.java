package frc.robot.subsystems.sensors;

import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDColor;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.Segment;
import org.littletonrobotics.junction.Logger;

public class CoralSensor extends SubsystemBase {
  private final Lights LED;
  private final RangeSensorIO io;
  protected final RangeSystemIOInputsAutoLogged inputs = new RangeSystemIOInputsAutoLogged();

  private final Segment indicator = new Segment(2, 2, 0);
  private RGBWColor indicatorColor = LEDColor.RED;
  private RGBWColor prevIndicatorColor = new RGBWColor();

  private final Alert disconnectedAlert =
      new Alert("Coral CANrange sensor disconnected.", AlertType.kError);

  public CoralSensor(Lights LED, RangeSensorIO io) {
    this.LED = LED;
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Sensors/Coral CANrange", inputs);
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
      // prevIndicatorColor = indicatorColor;  this assignment of object not working. 9/3
    }
  }

  public boolean isDetected() {
    return inputs.detected;
  }

  public double getDistance_mm() {
    return inputs.distance_mm;
  }
}
