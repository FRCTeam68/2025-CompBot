package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class LaserCanSystem extends SubsystemBase {
  private final String name;
  private final LaserCan lc;
  private final Alert connectionAlert;
  private final Alert measurementAlert;
  private double distance_mm;
  private LoggedTunableNumber threshold;

  public LaserCanSystem(String name, int can_id, double mythreshold) {
    this.name = name;

    lc = new LaserCan(can_id);

    connectionAlert = new Alert("LaserCan: " + name + ", configuration failed! ", AlertType.kError);

    measurementAlert =
        new Alert(
            "LaserCan: " + name + ", is out of range, or we can't get a reliable measurement! ",
            AlertType.kError);

    distance_mm = 0;
    threshold = new LoggedTunableNumber("LaserCan/" + name + "/threshold");
    threshold.initDefault(mythreshold);

    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in
    // GrappleHook
    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      // lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      connectionAlert.set(true);
      System.out.println("Configuration failed! " + e);
    }
  }

  @Override
  public void periodic() {
    distance_mm = -1;
    LaserCan.Measurement measurement = lc.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      distance_mm = measurement.distance_mm;
    }
    // else {
    //   measurementAlert.set(true);
    //   // You can still use distance_mm in here, if you're ok tolerating a clamped value or an
    //   // unreliable measurement.
    // }

    Logger.recordOutput("LaserCan/" + name + "/distance_mm", distance_mm);
  }

  public double getDistance_mm() {
    return distance_mm;
  }

  public boolean havePiece() {
    // coral sensor is measuring 0 because it is really close.
    // but definately measures nothing when no coral is there (i.e.)
    return (distance_mm >= 0 && distance_mm < 50);
  }
}
