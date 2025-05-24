package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;

public class TimedAlert {
  public static void set(String text, double seconds) {
    Alert alert = new Alert(text, AlertType.kInfo);
    Timer timer = new Timer();
    timer.start();
    alert.set(true);
  }
}
