package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
  private final DCMotorSim sim;
  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private PID[] PIDValues = new PID[3];

  private double appliedVoltage = 0.0;

  public ElevatorIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getFalcon500(2), .001, ElevatorIOTalonFX.getReduction()),
            DCMotor.getFalcon500(2));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      setVolts(0.0);
    } else {
      setVolts(controller.calculate(sim.getAngularPositionRotations()));
    }

    sim.update(Constants.loopPeriodSecs);
    inputs.connected = true;
    inputs.followerConnected = true;
    inputs.position = sim.getAngularPositionRotations();
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
    controller.setPID(PIDValues[slot].kP, PIDValues[slot].kI, PIDValues[slot].kD);
    controller.setSetpoint(position);
  }

  @Override
  public void stop() {
    setVolts(0.0);
  }

  @Override
  public void zero() {
    controller.setSetpoint(0);
    sim.setAngle(0);
  }

  @Override
  public void setPID(SlotConfigs... newconfig) {
    for (int i = 0; i < newconfig.length; i++) {
      PIDValues[i] = new PID(newconfig[i].kP, newconfig[i].kI, newconfig[i].kD);
    }
  }

  private class PID {
    double kP;
    double kI;
    double kD;

    private PID(double kP, double kI, double kD) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
    }
  }
}
