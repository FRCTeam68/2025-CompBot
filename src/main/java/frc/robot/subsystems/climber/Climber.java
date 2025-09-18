package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final String pathName = "Climber";
  private final ClimberIO io;
  protected final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final Alert disconnectedAlert =
      new Alert("Climber motor disconnected!", Alert.AlertType.kError);

  private LoggedTunableNumber kP0 = new LoggedTunableNumber(pathName + "/Slot 0 Configs/kP", 80);
  private LoggedTunableNumber kI0 = new LoggedTunableNumber(pathName + "/Slot 0 Configs/kI", 0);
  private LoggedTunableNumber kD0 = new LoggedTunableNumber(pathName + "/Slot 0 Configs/kD", 0);
  private LoggedTunableNumber kS0 = new LoggedTunableNumber(pathName + "/Slot 0 Configs/kS", 0);
  private LoggedTunableNumber kV0 = new LoggedTunableNumber(pathName + "/Slot 0 Configs/kV", 0);
  private LoggedTunableNumber kA0 = new LoggedTunableNumber(pathName + "/Slot 0 Configs/kA", 0);
  private LoggedTunableNumber kG0 = new LoggedTunableNumber(pathName + "/Slot 0 Configs/kG", 0);

  private LoggedTunableNumber mmV =
      new LoggedTunableNumber(pathName + "/Motion Magic Configs/Velocity", 70);
  private LoggedTunableNumber mmA =
      new LoggedTunableNumber(pathName + "/Motion Magic Configs/Acceleration", 140);
  private LoggedTunableNumber mmJ =
      new LoggedTunableNumber(pathName + "/Motion Magic Configs/Jerk", 500);

  private LoggedTunableNumber setpointBand =
      new LoggedTunableNumber(pathName + "/setpointBand", 0.03);

  @Getter private double setpoint = 0.0;

  public Climber(ClimberIO io) {
    this.io = io;

    setPID();
    io.setMotionMagic(
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(mmV.get())
            .withMotionMagicAcceleration(mmA.get())
            .withMotionMagicJerk(mmJ.get()));

    SmartDashboard.putData(pathName + "/Zero In Place", Commands.runOnce(() -> zero()));
    SmartDashboard.putData(
        pathName + "/Move to Zero Position", Commands.runOnce(() -> setPosition(0.0)));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    disconnectedAlert.set(!inputs.connected);
    Logger.recordOutput("Climber/At position", atPosition());

    // Visualize position
    Logger.recordOutput(
        "RobotPose/Climber",
        new Pose3d[] {
          new Pose3d(
              -0.2921,
              0,
              0.4398003396,
              new Rotation3d(
                  0,
                  Units.degreesToRadians(-47.3)
                      - Units.rotationsToRadians(inputs.positionRotations),
                  0))
        });

    // Update tunable numbers
    if (Constants.tuningMode) {
      if (kP0.hasChanged(hashCode())
          || kI0.hasChanged(hashCode())
          || kD0.hasChanged(hashCode())
          || kS0.hasChanged(hashCode())
          || kV0.hasChanged(hashCode())
          || kA0.hasChanged(hashCode())
          || kG0.hasChanged(hashCode())) {
        setPID();
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
   * Set goal position of the mechanism in rotations with slot 0
   *
   * @param position Goal position
   */
  public void setPosition(double position) {
    setPosition(position, 0);
  }

  /**
   * Set goal position of the mechanism in rotations
   *
   * @param position Goal position
   * @param slot Slot config number [0,2]
   */
  public void setPosition(double position, int slot) {
    setpoint = position;
    Logger.recordOutput(pathName + "/setpointPosition", setpoint);
    Logger.recordOutput(pathName + "/slot", slot);
    io.setPosition(position, slot);
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
            .withKP(kP0.get())
            .withKI(kI0.get())
            .withKD(kD0.get())
            .withKS(kS0.get())
            .withKV(kV0.get())
            .withKA(kA0.get())
            .withKG(kG0.get()));
  }

  public Command staticClimberCharacterization(double outputRampRate) {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              // stopProfile = true;
              timer.restart();
            },
            () -> {
              state.characterizationOutput = outputRampRate * timer.get();
              setVolts(state.characterizationOutput);
              Logger.recordOutput(
                  "Climber/StaticCharacterizationOutput", state.characterizationOutput);
            })
        .until(() -> getPosition() >= Constants.CLIMBER.DEPLOY)
        .finallyDo(
            () -> {
              // stopProfile = false;
              timer.stop();
              Logger.recordOutput("Climber/CharacterizationOutput", state.characterizationOutput);
            });
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }
}
