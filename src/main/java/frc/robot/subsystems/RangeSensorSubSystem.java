package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class RangeSensorSubSystem extends SubsystemBase {
  private final String name;
  private final CANrange can_range;
  private boolean sensorConnected;
  private boolean isDetected;
  private double distance_mm;
  private LoggedTunableNumber threshold;

  private final StatusSignal<Distance> distanceSignal;
  private final StatusSignal<Boolean> detectedSignal;

  private final Debouncer sensorConnectedDebounce = new Debouncer(0.5);

  public RangeSensorSubSystem(String name, int can_id, String bus, double mythreshold) {
    this.name = name;

    can_range = new CANrange(can_id, bus);
    sensorConnected = false;
    distance_mm = 0;
    isDetected = false;
    threshold = new LoggedTunableNumber("CAN_range/" + name + "/threshold");
    threshold.initDefault(mythreshold);

    distanceSignal = can_range.getDistance();
    detectedSignal = can_range.getIsDetected();

    tryUntilOk(
        5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, distanceSignal, detectedSignal));
    tryUntilOk(5, () -> can_range.optimizeBusUtilization(0, 1.0));
  }

  @Override
  public void periodic() {

    // Refresh all signals
    var sensorStatus = BaseStatusSignal.refreshAll(distanceSignal, detectedSignal);
    sensorConnected = sensorConnectedDebounce.calculate(sensorStatus.isOK());
    Logger.recordOutput("CAN_range/" + name + "/connected", sensorConnected);

    distance_mm = distanceSignal.getValue().in(Millimeters);
    Logger.recordOutput("CAN_range/" + name + "/distance_mm", distance_mm);

    isDetected = detectedSignal.getValue();
    Logger.recordOutput("CAN_range/" + name + "/isDetected", isDetected);

    // led status lights
    if (name == "intakeCoral") {
      if (isDetected) {
        LEDSegment.LED2.setColor(LightsSubsystem.blue);
      } else if (sensorConnected) {
        LEDSegment.LED2.setColor(LightsSubsystem.green);
      } else {
        LEDSegment.LED2.setColor(LightsSubsystem.red);
      }
    } else {
      if (isDetected) {
        LEDSegment.LED3.setColor(LightsSubsystem.blue);
      } else if (sensorConnected) {
        LEDSegment.LED3.setColor(LightsSubsystem.green);
      } else {
        LEDSegment.LED3.setColor(LightsSubsystem.red);
      }
    }
  }

  public double getDistance_mm() {
    return distance_mm;
  }

  public boolean havePiece() {
    return isDetected;
  }
}
