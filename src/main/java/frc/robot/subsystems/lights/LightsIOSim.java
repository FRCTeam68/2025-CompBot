package frc.robot.subsystems.lights;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.RGBWColor;
import frc.robot.Constants.LEDSegment;
import frc.robot.subsystems.lights.Lights.Segment;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class LightsIOSim implements LightsIO {
  private final String tableKey = "/Simulation/CANdle";
  private final String defaultHex = "#000000";

  // {Hex, Control, Name, Segment}
  private String[] integratedLEDValues = new String[8];
  private final String[] integratedLEDNames = {"0", "1", "2", "3", "4", "5", "6", "7"};
  private String[] otherLEDValues = new String[1];
  private final String[] otherLEDNames = {"All"};
  private final Segment[] otherLEDSegments = {LEDSegment.ALL};
  private Object[][] LEDValues = new Object[8 + otherLEDSegments.length][4];

  private LoggedNetworkString[] loggedLEDValues =
      new LoggedNetworkString[integratedLEDValues.length + otherLEDValues.length];

  public LightsIOSim() {
    for (int i = 0; i < 8; i++) {
      LEDValues[i] = new Object[] {defaultHex, "Solid", integratedLEDNames[i]};
      loggedLEDValues[i] =
          new LoggedNetworkString(
              tableKey + "/Integrated/" + i + " - " + integratedLEDNames[i],
              LEDValues[i][0].toString());
    }

    for (int i = 0; i < otherLEDSegments.length; i++) {
      LEDValues[i + 8] = new Object[] {defaultHex, "Solid", otherLEDNames[i], otherLEDSegments[i]};
      loggedLEDValues[i + 8] =
          new LoggedNetworkString(
              tableKey + "/Other/" + otherLEDNames[i], LEDValues[i + 8][0].toString());
    }

    // for (int i = 0; i < integratedLEDValues.length; i++) {
    //   integratedLEDValues[i] = "#000000";
    //   loggedLEDValues[i] =
    //       new LoggedNetworkString(
    //           tableKey + "/Integrated/" + i + " - " + integratedLEDNames[i],
    //           integratedLEDValues[i]);
    // }

    // for (int i = 0; i < otherLEDValues.length; i++) {
    //   otherLEDValues[i] = "#000000";
    //   loggedLEDValues[i + 8] =
    //       new LoggedNetworkString(tableKey + "/Other/" + otherLEDNames[i], otherLEDValues[i]);
    // }
  }

  @Override
  public void updateInputs(LightsIOInputs inputs) {
    inputs.connected = true;

    for (int i = 0; i < loggedLEDValues.length; i++) {
      loggedLEDValues[i].set(LEDValues[i][0].toString());
    }
  }

  @Override
  public void setControl(ControlRequest request) {
    Map<String, String> controlInfo = request.getControlInfo();
    String name = controlInfo.get("Name");
    if (name != "EmptyAnimation") {
      int startIndex = Integer.parseInt(request.getControlInfo().get("LEDStartIndex"));
      int endIndex = Integer.parseInt(request.getControlInfo().get("LEDEndIndex"));
      if (controlInfo.containsKey("Color")) {
        String colorHex = stringToHex(request.getControlInfo().get("Color"));
        for (int i = startIndex; i <= Math.min(7, endIndex); i++) {
          LEDValues[i][0] = colorHex;
        }
      }
    }
  }

  private String stringToHex(String colorString) {
    String[] splitColors = colorString.substring(5).replace(")", "").split(",");
    int red = Integer.parseInt(splitColors[0].trim());
    int green = Integer.parseInt(splitColors[1].trim());
    int blue = Integer.parseInt(splitColors[2].trim());
    int white = Integer.parseInt(splitColors[3].trim());

    RGBWColor color = new RGBWColor(red, green, blue, white);

    return color.toHexString().substring(0, 7);
  }
}
