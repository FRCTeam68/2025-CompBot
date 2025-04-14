package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class WristIOSim implements WristIO {
  private final DCMotorSim sim;
  private double appliedVoltage = 0.0;

  public WristIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), .1, WristIOTalonFX.reduction),
            DCMotor.getKrakenX60Foc(1));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      setVolts(0.0);
    }

    inputs.connected = true;
    sim.update(Constants.loopPeriodSecs);
    inputs.positionRotations = sim.getAngularPositionRotations();
    inputs.velocityRotsPerSec = sim.getAngularVelocityRPM() * 60;
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setPosition(double position, int slot) {
    sim.setAngle(Units.rotationsToRadians(position));
  }

  @Override
  public void stop() {
    setVolts(0.0);
  }

  @Override
  public void zero() {
    sim.setAngle(0);
  }
}
