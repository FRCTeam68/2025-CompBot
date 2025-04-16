package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final String pathName = "Elevator";
  private final ElevatorIO io;
  protected final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final Alert disconnected;

  private LoggedTunableNumber kP = new LoggedTunableNumber(pathName + "/kP", 10);
  private LoggedTunableNumber kI = new LoggedTunableNumber(pathName + "/kI", 0);
  private LoggedTunableNumber kD = new LoggedTunableNumber(pathName + "/kD", 0);
  private LoggedTunableNumber kS = new LoggedTunableNumber(pathName + "/kS", 0.5);
  private LoggedTunableNumber kV = new LoggedTunableNumber(pathName + "/kV", 0.2);
  private LoggedTunableNumber kA = new LoggedTunableNumber(pathName + "/kA", 0);
  private LoggedTunableNumber kG = new LoggedTunableNumber(pathName + "/kG", 0);

  private LoggedTunableNumber mmV = new LoggedTunableNumber(pathName + "/mmV", 40);
  private LoggedTunableNumber mmA = new LoggedTunableNumber(pathName + "/mmA", 120);
  private LoggedTunableNumber mmJ = new LoggedTunableNumber(pathName + "/mmJ", 400);

  private LoggedTunableNumber setpointBand =
      new LoggedTunableNumber(pathName + "/setpointBand", 0.005);

  @Getter private double setpoint = 0.0;

  public Elevator(ElevatorIO io) {
    this.io = io;

    io.setPID(
        new SlotConfigs()
            .withKP(kP.get())
            .withKI(kI.get())
            .withKD(kD.get())
            .withKS(kS.get())
            .withKV(kV.get())
            .withKA(kA.get())
            .withKG(kG.get()));
    io.setMotionMagic(
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(mmV.get())
            .withMotionMagicAcceleration(mmA.get())
            .withMotionMagicJerk(mmJ.get()));

    disconnected = new Alert("Elevator" + " motor disconnected!", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    disconnected.set(!inputs.connected);

    // Update tunable numbers
    if (Constants.tuningMode) {
      if (kP.hasChanged(hashCode())
          || kI.hasChanged(hashCode())
          || kD.hasChanged(hashCode())
          || kS.hasChanged(hashCode())
          || kV.hasChanged(hashCode())
          || kA.hasChanged(hashCode())
          || kG.hasChanged(hashCode())) {
        SlotConfigs newconfig = new SlotConfigs();
        newconfig.kP = kP.get();
        newconfig.kI = kI.get();
        newconfig.kD = kD.get();
        newconfig.kS = kS.get();
        newconfig.kV = kV.get();
        newconfig.kA = kA.get();
        newconfig.kG = kG.get();
        io.setPID(newconfig);
      }

      if (mmV.hasChanged(hashCode()) || mmA.hasChanged(hashCode()) || mmJ.hasChanged(hashCode())) {
        MotionMagicConfigs newMMconfig = new MotionMagicConfigs();
        newMMconfig.MotionMagicCruiseVelocity = mmV.get();
        newMMconfig.MotionMagicAcceleration = mmA.get();
        newMMconfig.MotionMagicJerk = mmJ.get();
        io.setMotionMagic(newMMconfig);
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
    Logger.recordOutput(pathName + "/setpointVolts", setpoint);
    io.setVolts(inputVolts);
  }

  /**
   * Set goal position of the mechanism in inches
   *
   * @param position goal position
   */
  public void setPosition(double position) {
    setpoint = position;
    Logger.recordOutput(pathName + "/setpointPosition", setpoint);
    io.setPosition(position, 0);
  }

  /**
   * Velocity of the mechanism in inches per second
   *
   * @return Velocity
   */
  public double getSpeed() {
    return inputs.velocityInchesPerSec;
  }

  /**
   * Position of the mechanism in inches
   *
   * @return Position
   */
  public double getPosition() {
    return inputs.positionInches;
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
    return Math.abs(setpoint - getPosition()) < setpointBand.getAsDouble();
  }

  /** Stop motor */
  public void stop() {
    io.stop();
  }

  /** Sets the current mechanism position to zero */
  public void zero() {
    io.zero();
  }
}
