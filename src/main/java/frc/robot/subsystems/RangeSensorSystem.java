package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class RangeSensorSystem extends SubsystemBase {
  private final String name;
  private final CANrange can_range;
  private final Alert connectionAlert;
  private final Alert measurementAlert;
  private final boolean isDetected;
  private double distance_mm;
  private LoggedTunableNumber threshold;

  public RangeSensorSystem(String name, int can_id, double mythreshold) {
    this.name = name;

    can_range = new CANrange(can_id, "rio");

    connectionAlert =
        new Alert("can_range: " + name + ", configuration failed! ", AlertType.kError);

    measurementAlert =
        new Alert(
            "can_range: " + name + ", is out of range, or we can't get a reliable measurement! ",
            AlertType.kError);

    distance_mm = 0;
    threshold = new LoggedTunableNumber("CAN_range/" + name + "/threshold");
    threshold.initDefault(mythreshold);

    isDetected = can_range.getIsDetected().getValue();
  }

  @Override
  public void periodic() {
    distance_mm = -1;
    distance_mm = can_range.getDistance().getValueAsDouble();

    Logger.recordOutput("CAN_range/" + name + "/distance_mm", distance_mm);
  }

  public double getDistance_mm() {
    return distance_mm;
  }

  public boolean havePiece() {
    // coral sensor is measuring 0 because it is really close.
    // but definately measures nothing when no coral is there (i.e.)
    return (can_range.getIsDetected().getValue());
  }
}
