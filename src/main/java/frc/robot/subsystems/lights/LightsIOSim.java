package frc.robot.subsystems.lights;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class LightsIOSim implements LightsIO {
  private static final String tableKey = "/Simulation/CANdle";

  private static String[] integratedLEDValues = new String[8];
  private static String[] lastIntegratedLEDValues = new String[8];

  public LightsIOSim() {
    for (int i = 0; i < integratedLEDValues.length; i++) {
      integratedLEDValues[i] = "#000000";
    }
  }

  @Override
  public void updateInputs(LightsIOInputs inputs) {
    inputs.connected = true;
    if (integratedLEDValues != lastIntegratedLEDValues) {
      for (int j = 0; j < integratedLEDValues.length; j++) {
        new LoggedNetworkString(tableKey + "/" + String.valueOf(j), integratedLEDValues[j]);
      }
      lastIntegratedLEDValues = integratedLEDValues;
    }
  }

  @Override
  public void setControl(ControlRequest request) {
    Map<String, String> controlInfo = request.getControlInfo();
    String name = controlInfo.get("Name");
    if (name != "EmptyAnimation") {
      int startIndex = Integer.parseInt(request.getControlInfo().get("LEDStartIndex"));
      SmartDashboard.putNumber("test1", startIndex);
      int endIndex = Integer.parseInt(request.getControlInfo().get("LEDEndIndex"));
      SmartDashboard.putNumber("test2", endIndex);
      String color = colorStringToHex(request.getControlInfo().get("Color"));
      SmartDashboard.putString("test3", color);
      integratedLEDValues[1] = color;
      if (startIndex <= 7) {
        for (int i = startIndex; i <= Math.min(7, endIndex); i++) {
          integratedLEDValues[i] = color;
        }
      }
    }
  }
  // RGBW(255, 45,5,0)
  private String colorStringToHex(String color) {
    String[] splitColors = color.substring(5).replace(")", "").split(",");
    int red = Integer.parseInt(splitColors[0].trim());
    int green = Integer.parseInt(splitColors[1].trim());
    int blue = Integer.parseInt(splitColors[2].trim());

    StringBuilder hex = new StringBuilder(7);
    hex.append('#');
    hex.append(nibbleToHex((red >> 4) & 0xF));
    hex.append(nibbleToHex(red & 0xF));
    hex.append(nibbleToHex((green >> 4) & 0xF));
    hex.append(nibbleToHex(green & 0xF));
    hex.append(nibbleToHex((blue >> 4) & 0xF));
    hex.append(nibbleToHex(blue & 0xF));
    return hex.toString();
  }

  private static char nibbleToHex(int nibble) {
    if (nibble < 10) {
      return (char) (nibble + '0');
    } else {
      return (char) (nibble - 10 + 'A');
    }
  }
}
