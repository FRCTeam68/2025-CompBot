package frc.robot.subsystems.sensors2;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.PhoenixUtil;

public class RangeSensorIOCANrange implements RangeSensorIO {
  private final CANrange canrange;

  private final StatusSignal<Boolean> detected;
  private final StatusSignal<Distance> distance;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public RangeSensorIOCANrange(Integer id, String bus, CANrangeConfiguration config) {
    canrange = new CANrange(id, bus);

    tryUntilOk(5, () -> canrange.getConfigurator().apply(config));

    detected = canrange.getIsDetected();
    distance = canrange.getDistance();

    tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, detected, distance));
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(canrange));
    PhoenixUtil.registerSignals((bus == "rio") ? false : true, detected, distance);
  }

  @Override
  public void updateInputs(RangeSystemIOInputs inputs) {
    inputs.connected = connectedDebouncer.calculate(BaseStatusSignal.isAllGood(detected, distance));
    inputs.detected = detected.getValue();
    inputs.distance_mm = distance.getValueAsDouble();
  }
}
