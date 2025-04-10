package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDColor;
import frc.robot.subsystems.LightsSubsystem.Color;
import frc.robot.subsystems.LightsSubsystem.Segment;
import org.littletonrobotics.junction.Logger;

public class RangeSensorSubSystem extends SubsystemBase {
  private final String folderName = "CANrange/";
  private final String name;
  private final Segment indicator;
  private final CANrange canrange;

  private boolean isConnected = false;
  private boolean prevIsConnected = false;
  private boolean isDetected = false;
  private boolean prevIsDetected = false;
  private double distance_mm = 0;
  private Color indicatorColor = LEDColor.RED;
  private Color prevIndicatorColor = LEDColor.RED;
  private boolean initialLog = true;

  private final StatusSignal<Distance> distanceSignal;
  private final StatusSignal<Boolean> detectedSignal;

  private final Debouncer sensorConnectedDebounce = new Debouncer(0.5);

  public RangeSensorSubSystem(CANrangeConstants constants) {
    this.name = constants.name;
    this.indicator = constants.indicator;

    canrange = new CANrange(constants.canID, constants.bus);

    distanceSignal = canrange.getDistance();
    detectedSignal = canrange.getIsDetected();

    tryUntilOk(5, () -> canrange.getConfigurator().apply(constants.config));
    tryUntilOk(
        5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, distanceSignal, detectedSignal));
    tryUntilOk(5, () -> canrange.optimizeBusUtilization(0, 1.0));
  }

  /**
   * Creates the constants for a CANrange sensor.
   *
   * @param name Logged name of CANrange.
   * @param canID CAN ID of CANrange.
   * @param bus CAN bus name.
   * @param indicator LED segment for indication.
   * @param config Standard CANrange configuration.
   */
  public static class CANrangeConstants {
    public String name;
    public int canID;
    public String bus;
    public Segment indicator;
    public CANrangeConfiguration config;

    public CANrangeConstants(
        String name, int canID, String bus, Segment indicator, CANrangeConfiguration config) {
      this.name = name;
      this.canID = canID;
      this.bus = bus;
      this.indicator = indicator;
      config.FutureProofConfigs = true;
      this.config = config;
    }
  }

  @Override
  public void periodic() {

    // Refresh all signals
    var sensorStatus = BaseStatusSignal.refreshAll(distanceSignal, detectedSignal);
    isConnected = sensorConnectedDebounce.calculate(sensorStatus.isOK());
    isDetected = detectedSignal.getValue();
    distance_mm = distanceSignal.getValue().in(Millimeters);

    // log signals
    if (isConnected != prevIsConnected || initialLog) {
      Logger.recordOutput(folderName + name + "/connected", isConnected);
      prevIsConnected = isConnected;
    }

    if (isDetected != prevIsDetected || initialLog) {
      Logger.recordOutput(folderName + name + "/isDetected", isDetected);
      prevIsDetected = isDetected;
    }

    Logger.recordOutput(folderName + name + "/distance_mm", distance_mm);

    // determine indicator color
    if (isDetected) {
      indicatorColor = LEDColor.BLUE;
    } else if (isConnected) {
      indicatorColor = LEDColor.GREEN;
    } else {
      indicatorColor = LEDColor.RED;
    }

    // set indicator
    if (indicatorColor != prevIndicatorColor || initialLog) {
      LightsSubsystem.setColor(indicatorColor, indicator);
      prevIndicatorColor = indicatorColor;
    }

    if (initialLog) initialLog = false;
  }

  /** 
   * Get the distance of the CANrange in millimeters.
   * 
   * @return Distance in millimeters
   */
  public double getDistance_mm() {
    return distance_mm;
  }

  /** 
   * Get the detected status of the CANrange.
   * 
   * @return Detected status
   */
  public boolean isDetected() {
    return isDetected;
  }
}
