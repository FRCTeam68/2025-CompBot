package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
  private final DCMotorSim sim;
  private double appliedVoltage = 0.0;

  public ElevatorIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(2), .1, ElevatorIOTalonFX.reduction),
            DCMotor.getKrakenX60Foc(2));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      setVolts(0.0);
    }

    inputs.connected = true;
    sim.update(Constants.loopPeriodSecs);
    inputs.positionInches =
        sim.getAngularPositionRotations() * Math.PI * ElevatorIOTalonFX.pitchDiameter;
    inputs.positionRotations = sim.getAngularPositionRotations();
    inputs.velocityInchesPerSec =
        sim.getAngularVelocityRPM() * 60 * Math.PI * ElevatorIOTalonFX.pitchDiameter;
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
    sim.setAngle(Units.rotationsToRadians(position / (Math.PI * ElevatorIOTalonFX.pitchDiameter)));
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
