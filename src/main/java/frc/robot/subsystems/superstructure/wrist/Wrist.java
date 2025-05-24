package frc.robot.subsystems.superstructure.wrist;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  protected final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert =
      new Alert("Wrist motor disconnected!", AlertType.kError);
  private final Alert cancoderDisconnectedAlert =
      new Alert("Wrist CANcoder disconnected!", AlertType.kWarning);

  private LoggedTunableNumber kP0 = new LoggedTunableNumber("Wrist/Slot0Configs/kP", 120);
  private LoggedTunableNumber kD0 = new LoggedTunableNumber("Wrist/Slot0Configs/kD", 0);
  private LoggedTunableNumber kS0 = new LoggedTunableNumber("Wrist/Slot0Configs/kS", 0);
  private LoggedTunableNumber kV0 = new LoggedTunableNumber("Wrist/Slot0Configs/kV", 0);
  private LoggedTunableNumber kA0 = new LoggedTunableNumber("Wrist/Slot0Configs/kA", 0);
  private LoggedTunableNumber kG0 = new LoggedTunableNumber("Wrist/Slot0Configs/kG", 0);

  private LoggedTunableNumber kP1 = new LoggedTunableNumber("Wrist/Slot1Configs/kP", 50);
  private LoggedTunableNumber kD1 = new LoggedTunableNumber("Wrist/Slot1Configs/kD", 0);
  private LoggedTunableNumber kS1 = new LoggedTunableNumber("Wrist/Slot1Configs/kS", 0);
  private LoggedTunableNumber kV1 = new LoggedTunableNumber("Wrist/Slot1Configs/kV", 0);
  private LoggedTunableNumber kA1 = new LoggedTunableNumber("Wrist/Slot1Configs/kA", 0);
  private LoggedTunableNumber kG1 = new LoggedTunableNumber("Wrist/Slot1Configs/kG", 0);

  private LoggedTunableNumber mmV =
      new LoggedTunableNumber("Wrist/MotionMagicConfigs/Velocity", 50);
  private LoggedTunableNumber mmA =
      new LoggedTunableNumber("Wrist/MotionMagicConfigs/Acceleration", 70);
  private LoggedTunableNumber mmJ = new LoggedTunableNumber("Wrist/MotionMagicConfigs/Jerk", 400);

  private LoggedTunableNumber setpointBand = new LoggedTunableNumber("Wrist/SetpointBand", 0.005);

  @Getter private double setpoint = 0.0;

  private LoggedNetworkNumber testSetpoint =
      new LoggedNetworkNumber("SmartDashboard/Wrist/RunToPosition/Setpoint", 0);
  private LoggedNetworkNumber testSlot =
      new LoggedNetworkNumber("SmartDashboard/Wrist/RunToPosition/Slot", 0);

  public Wrist(WristIO io) {
    this.io = io;
    setPID();
    setMotionMagic();

    // Dashboard tuning commands
    SmartDashboard.putData("Wrist/ZeroInPlace", Commands.runOnce(() -> zero()));
    SmartDashboard.putData(
        "Wrist/RunToPosition/Run",
        Commands.runOnce(() -> setPosition(testSetpoint.get(), (int) testSlot.get())));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    motorDisconnectedAlert.set(!inputs.connected);
    cancoderDisconnectedAlert.set(!inputs.CANcoderConnected);
    Logger.recordOutput("Wrist/AtPosition", atPosition());

    // Update tunable numbers
    if (Constants.tuningMode) {
      if (kP0.hasChanged(hashCode())
          || kD0.hasChanged(hashCode())
          || kS0.hasChanged(hashCode())
          || kV0.hasChanged(hashCode())
          || kA0.hasChanged(hashCode())
          || kG0.hasChanged(hashCode())
          || kP1.hasChanged(hashCode())
          || kD1.hasChanged(hashCode())
          || kS1.hasChanged(hashCode())
          || kV1.hasChanged(hashCode())
          || kA1.hasChanged(hashCode())
          || kG1.hasChanged(hashCode())) {
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
    Logger.recordOutput("Wrist/SetpointVolts", setpoint);
    io.setVolts(inputVolts);
  }

  /**
   * Set goal position of the mechanism in rotations
   *
   * @param position Goal position
   * @param slot Slot config number [0,2]
   */
  public void setPosition(double position, int... slot) {
    int activeSlot = slot.length > 0 ? slot[0] : 0;
    setpoint = position;
    Logger.recordOutput("Wrist/SetpointPosition", setpoint);
    Logger.recordOutput("Wrist/Slot", activeSlot);
    io.setPosition(position, activeSlot);
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
   * @return Position
   */
  public double getPosition() {
    return inputs.positionRotations;
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

  private void setPID() {
    io.setPID(
        new SlotConfigs()
            .withKP(kP0.getAsDouble())
            .withKD(kD0.getAsDouble())
            .withKS(kS0.getAsDouble())
            .withKV(kV0.getAsDouble())
            .withKA(kA0.getAsDouble())
            .withKG(kG0.getAsDouble()),
        new SlotConfigs()
            .withKP(kP1.getAsDouble())
            .withKD(kD1.getAsDouble())
            .withKS(kS1.getAsDouble())
            .withKV(kV1.getAsDouble())
            .withKA(kA1.getAsDouble())
            .withKG(kG1.getAsDouble()));
  }

  private void setMotionMagic() {
    io.setMotionMagic(
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(mmV.getAsDouble())
            .withMotionMagicAcceleration(mmA.getAsDouble())
            .withMotionMagicJerk(mmJ.getAsDouble()));
  }
}
