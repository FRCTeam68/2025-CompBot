package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class RollerSubSystem extends SubsystemBase {

  public enum Mode {
    VOLTAGE_OUT,
    VOLTAGE_FOC,
    CURRENTTORQUE_FOC
  }

  private String m_name;
  private Mode m_presentMode;
  private double m_setPoint_Speed;
  private double m_setPoint_Voltage;
  private TalonFX m_rollerMotor;
  private VoltageOut m_voltageOut;
  private VelocityVoltage m_voltageVelocity;
  private VelocityTorqueCurrentFOC m_torqueVelocity;
  private NeutralOut m_brake;

  public RollerSubSystem(String name, int canID, String canbus, boolean inverted) {
    m_name = name;
    m_presentMode = Mode.CURRENTTORQUE_FOC;
    m_setPoint_Speed = 0;

    m_rollerMotor = new TalonFX(canID, canbus);

    m_voltageOut = new VoltageOut(0);

    /* Start at velocity 0, enable FOC, no feed forward, use slot 0 */
    m_voltageVelocity = new VelocityVoltage(0);
    /* Start at velocity 0, no feed forward, use slot 1 */
    m_torqueVelocity = new VelocityTorqueCurrentFOC(0);
    /* Keep a neutral out so we can disable the motor */
    m_brake = new NeutralOut();
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.Inverted =
        inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI =
        0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD =
        0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second

    // configs.Slot0.kP = .2;
    // configs.Slot0.kI = 0;
    // configs.Slot0.kD = 0;
    // configs.Slot0.kV = 0;

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    configs.CurrentLimits.StatorCurrentLimit = 60;

    /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
    configs.Slot1.kI =
        0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    configs.Slot1.kD =
        0.001; // A change of 1000 rotation per second squared results in 1 amp output

    // Peak output of 40 amps
    // configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    // configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_rollerMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println(
          "Could not apply configs to " + m_name + ", error code: " + status.toString());
    }

    m_rollerMotor.setControl(m_voltageOut.withOutput(0));

    Logger.recordOutput(m_name + "/Comment", m_name + " subsystem created");
  }

  public void setSpeedVout(double desiredVoltage) {
    if (m_presentMode == Mode.VOLTAGE_OUT) {
      if (Math.abs(desiredVoltage) <= Constants.ROLLER.MAX_VOLTAGE * .1) { // Joystick deadzone
        m_setPoint_Voltage = 0;
      } else {
        m_setPoint_Voltage = desiredVoltage;
      }
      System.out.println("              desired voltage: " + m_setPoint_Voltage);
      Logger.recordOutput(m_name + "/setVoltage", m_setPoint_Voltage);
      m_rollerMotor.setControl(m_voltageOut.withOutput(m_setPoint_Voltage));
    }
  }

  public void bumpSpeed(double bumpAmount) {
    double new_value = m_setPoint_Speed + bumpAmount;
    if (Math.abs(new_value) < 1) {
      new_value = bumpAmount < 0 ? -1 : 1;
    }
    setSpeed(new_value);
  }

  public void setSpeed(double desiredRotationsPerSecond) {

    System.out.println("    set " + m_name + " desired speed: " + desiredRotationsPerSecond);

    if (Math.abs(desiredRotationsPerSecond) < 1) { // Joystick deadzone
      desiredRotationsPerSecond = 0;
      m_setPoint_Speed = 0;
      /* Disable the motor instead */
      m_rollerMotor.setControl(m_brake);
    } else {
      if (desiredRotationsPerSecond > Constants.ROLLER.MAX_SPEED) {
        m_setPoint_Speed = Constants.ROLLER.MAX_SPEED;
        System.out.println("  trimmed to max speed: " + m_setPoint_Speed);
        Logger.recordOutput(m_name + "/Comment", "  trimmed to max speed: " + m_setPoint_Speed);
      } else if (desiredRotationsPerSecond < -Constants.ROLLER.MAX_SPEED) {
        m_setPoint_Speed = -Constants.ROLLER.MAX_SPEED;
        System.out.println("  trimmed to max speed: " + m_setPoint_Speed);
        Logger.recordOutput(m_name + "/Comment", "  trimmed to min speed: " + m_setPoint_Speed);
      } else {
        m_setPoint_Speed = desiredRotationsPerSecond;
      }

      Logger.recordOutput(m_name + "/setSpeed", m_setPoint_Speed);

      switch (m_presentMode) {
        case VOLTAGE_OUT:
          this.setSpeedVout(
              (m_setPoint_Speed / Constants.ROLLER.MAX_SPEED) * Constants.ROLLER.MAX_VOLTAGE);
          break;
        default:
        case VOLTAGE_FOC:
          /* Use voltage velocity */
          m_rollerMotor.setControl(m_voltageVelocity.withVelocity(m_setPoint_Speed));

          break;

        case CURRENTTORQUE_FOC:
          double friction_torque =
              (desiredRotationsPerSecond > 0)
                  ? 1
                  : -1; // To account for friction, we add this to the arbitrary feed forward
          /* Use torque velocity */
          m_rollerMotor.setControl(
              m_torqueVelocity.withVelocity(m_setPoint_Speed).withFeedForward(friction_torque));
          break;
      }
    }
  }

  public double getSpeed() {
    return this.m_setPoint_Speed;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType(m_name);
    builder.addDoubleProperty("setpoint speed", this::getSpeed, this::setSpeed);
    builder.addStringProperty("Mode", () -> m_presentMode.toString(), null);
  }
}
