package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  protected final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final Alert disconnected;
  private final Alert followerDisconnected;

  private LoggedTunableNumber kP0 = new LoggedTunableNumber("Elevator/Slot0Configs/kP", 5);
  private LoggedTunableNumber kD0 = new LoggedTunableNumber("Elevator/Slot0Configs/kD", 0);
  private LoggedTunableNumber kS0 = new LoggedTunableNumber("Elevator/Slot0Configs/kS", 0.5);
  private LoggedTunableNumber kV0 = new LoggedTunableNumber("Elevator/Slot0Configs/kV", 0.2);
  private LoggedTunableNumber kA0 = new LoggedTunableNumber("Elevator/Slot0Configs/kA", 0);
  private LoggedTunableNumber kG0 = new LoggedTunableNumber("Elevator/Slot0Configs/kG", 0.5);

  private LoggedTunableNumber mmV =
      new LoggedTunableNumber("Elevator/MotionMagicConfigs/Velocity", 40);
  private LoggedTunableNumber mmA =
      new LoggedTunableNumber("Elevator/MotionMagicConfigs/Acceleration", 120);
  private LoggedTunableNumber mmJ =
      new LoggedTunableNumber("Elevator/MotionMagicConfigs/Jerk", 400);

  private LoggedTunableNumber setpointBand = new LoggedTunableNumber("Elevator/SetpointBand", 0.2);

  @Getter private double setpoint = 0.0;

  private LoggedNetworkNumber testSetpoint =
      new LoggedNetworkNumber("SmartDashboard/Elevator/RunToPosition/Setpoint", 0);

  public Elevator(ElevatorIO io) {
    this.io = io;

    setPID();
    setMotionMagic();

    disconnected = new Alert("Lead elevator motor disconnected.", Alert.AlertType.kError);
    followerDisconnected =
        new Alert("Follower elevator motor disconnected.", Alert.AlertType.kError);

    // Dashboard tuning commands
    SmartDashboard.putData("Elevator/ZeroInPlace", Commands.runOnce(() -> zero()));
    SmartDashboard.putData(
        "Elevator/RunToPosition/Run", Commands.runOnce(() -> setPosition(testSetpoint.get())));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    disconnected.set(!inputs.connected);
    followerDisconnected.set(!inputs.followerConnected);
    Logger.recordOutput("Elevator/AtPosition", atPosition());

    // Update tunable numbers
    if (Constants.tuningMode) {
      if (kP0.hasChanged(hashCode())
          || kD0.hasChanged(hashCode())
          || kS0.hasChanged(hashCode())
          || kV0.hasChanged(hashCode())
          || kA0.hasChanged(hashCode())
          || kG0.hasChanged(hashCode())) {
        setPID();
      }

      if (mmV.hasChanged(hashCode()) || mmA.hasChanged(hashCode()) || mmJ.hasChanged(hashCode())) {
        setMotionMagic();
      }
    }
  }

  public void setAtSetpointBand(LoggedTunableNumber band) {
    setpointBand = band;
  }

  /**
   * Set applied voltage to the motor
   *
   * @param inputVolts Voltage to drive at
   */
  public void setVolts(double inputVolts) {
    setpoint = inputVolts;
    Logger.recordOutput("Elevator/setpointVolts", setpoint);
    io.setVolts(inputVolts);
  }

  /**
   * Set goal position of the mechanism in rotations
   *
   * @param position Goal position
   */
  public void setPosition(double position) {
    setpoint = position;
    Logger.recordOutput("Elevator/setpointPosition", setpoint);
    io.setPosition(position, 0);
  }

  /**
   * Velocity of the mechanism in rotations per second
   *
   * @return Velocity
   */
  public double getSpeed() {
    return inputs.velocityRotsPerSec;
  }

  /**
   * Position of the mechanism in rotations
   *
   * @return Position rotations
   */
  public double getPositionRotations() {
    return inputs.position;
  }

  /**
   * Position of the mechanism in meters
   *
   * @return Position meters
   */
  public double getPositionMeters() {
    return Units.inchesToMeters(
        inputs.position * ElevatorIOTalonFX.getSpoolDiameter() * Math.PI * 2);
  }

  /**
   * Current corresponding to the torque output by the lead motor. Similar to StatorCurrent. Users
   * will likely prefer this current to calculate the applied torque to the rotor.
   *
   * <p>Stator current where positive current means torque is applied in the forward direction as
   * determined by the Inverted setting.
   *
   * @return Lead motor torque current
   */
  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }

  /**
   * Current corresponding to the torque output by the follower motor. Similar to StatorCurrent.
   * Users will likely prefer this current to calculate the applied torque to the rotor.
   *
   * <p>Stator current where positive current means torque is applied in the forward direction as
   * determined by the Inverted setting.
   *
   * @return Follower motor torque current
   */
  public double getFollowerTorqueCurrent() {
    return inputs.followerTorqueCurrentAmps;
  }

  /**
   * Check if mechanism is at goal position with error setpointBand
   *
   * @return True if mechanism is at goal position, false otherwise
   */
  public boolean atPosition() {
    return Math.abs(setpoint - inputs.position) < setpointBand.getAsDouble();
  }

  /** Stop motor */
  public void stop() {
    io.stop();
  }

  /** Sets the current mechanism position to zero */
  public void zero() {
    io.zero();
  }

  private void setPID() {
    io.setPID(
        new SlotConfigs()
            .withKP(kP0.getAsDouble())
            .withKD(kD0.getAsDouble())
            .withKS(kS0.getAsDouble())
            .withKV(kV0.getAsDouble())
            .withKA(kA0.getAsDouble())
            .withKG(kG0.getAsDouble()));
  }

  private void setMotionMagic() {
    io.setMotionMagic(
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(mmV.getAsDouble())
            .withMotionMagicAcceleration(mmA.getAsDouble())
            .withMotionMagicJerk(mmJ.getAsDouble()));
  }
}
