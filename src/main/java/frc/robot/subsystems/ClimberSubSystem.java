package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

// --45 rotations/s at full throttle.
// --left motor 98 rotation range
// --right motor 107 rotation range and needs negative voltage
// --distance traveled is 9"
// --45:1 (3:3:5) gear ratio
// --spool diameter 1.25"

public class ClimberSubSystem extends SubsystemBase {

  private double m_setPoint_Left_Speed;
  private double m_setPoint_Right_Speed;
  private TalonFX m_climberLeftMotor;
  private TalonFX m_climberRightMotor;
  private VoltageOut m_voltageOut;
  private NeutralOut m_neutral;
  private double m_setPoint_Left_Voltage;
  private double m_setPoint_Right_Voltage;
  private double m_leftPosition;
  private double m_rightPosition;
  private boolean m_pitMode;

  public ClimberSubSystem() {
    m_setPoint_Left_Speed = 0;
    m_setPoint_Right_Speed = 0;
    m_leftPosition = 0;
    m_rightPosition = 0;

    setPitMode(false);

    climberMotorsInit();
  }

  private void climberMotorsInit() {
    m_climberLeftMotor = new TalonFX(Constants.CLIMBER.LEFT_CANID, Constants.CLIMBER.CANBUS);
    m_climberRightMotor = new TalonFX(Constants.CLIMBER.RIGHT_CANID, Constants.CLIMBER.CANBUS);

    // m_climberLeftMotor.setInverted(true);
    // m_climberRightMotor.setInverted(false);  // pick CW versus CCW

    m_voltageOut = new VoltageOut(0);

    /* Keep a neutral out so we can disable the motor */
    m_neutral = new NeutralOut();

    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.Feedback.SensorToMechanismRatio = 1.0;

    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    // Peak output of 40 amps
    // configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    // configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // configs.withCurrentLimits(Constants.limit60);
    configs.CurrentLimits.StatorCurrentLimit = 60;

    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    // configs.CurrentLimits.SupplyCurrentLimit = 30.0;

    // configs.CurrentLimits.SupplyTimeThreshold = 0.01;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_climberLeftMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println(
          "Could not apply configs to left climber motor, error code: " + status.toString());
    }

    // Right motor needs negative voltage.  Invert it so position reports positive
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    /* Retry config apply up to 5 times, report if failure */
    for (int i = 0; i < 5; ++i) {
      status = m_climberRightMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println(
          "Could not apply configs to right climber motor, error code: " + status.toString());
    }

    System.out.println("climber subsystem created");
    Logger.recordOutput("Climber/Comment", "subsystem created");
  }

  public void setSpeedVout(double leftDesiredVoltage, double rightDesiredVoltage) {
    setLeftSpeedVout(leftDesiredVoltage);
    setRightSpeedVout(rightDesiredVoltage);
  }

  public void setLeftSpeedVout(double desiredVoltage) {
    // System.out.println("left climber setSpeedesired" + desiredVoltage);

    m_leftPosition = m_climberLeftMotor.getPosition().getValueAsDouble();

    if ((Math.abs(desiredVoltage) <= 1)) {
      // deadzone for motor control, just make it zero volts
      // System.out.println("set left zero");
      m_setPoint_Left_Voltage = 0;
    } else if ((desiredVoltage < -1) && (m_leftPosition < -Constants.CLIMBER.MAX_HEIGHT)) {
      // asking to travel up, but already at max height
      System.out.println("left climber max height reached");
      m_setPoint_Left_Voltage = 0;
    } else if ((desiredVoltage > 1) && (m_leftPosition >= 0)) {
      // asking to travel down, but already at min height
      //    resetMode active needs negative values to respool climber in the pits
      //    because when you turn robot on position will be zero.
      //    So do not limit at zero.
      if (m_pitMode) {
        // go slower
        m_setPoint_Left_Voltage = desiredVoltage / 2;
      } else {
        System.out.println("left climber start height 0 reached");
        m_setPoint_Left_Voltage = 0;
      }
    } else {
      m_setPoint_Left_Voltage = m_pitMode ? desiredVoltage / 2 : desiredVoltage;
      ;
    }

    // System.out.println("left climber setSpeedVout: " + m_setPoint_Left_Voltage);
    Logger.recordOutput("Climber/LeftPos", m_leftPosition);
    Logger.recordOutput("Climber/LeftVolt", m_setPoint_Left_Voltage);
    m_climberLeftMotor.setControl(m_voltageOut.withOutput(m_setPoint_Left_Voltage));
  }

  public void setRightSpeedVout(double desiredVoltage) {
    // System.out.println("right climber setSpeedesired" + desiredVoltage);

    m_rightPosition = m_climberRightMotor.getPosition().getValueAsDouble();

    if ((Math.abs(desiredVoltage) <= 1)) {
      // deadzone for motor control, just make it zero volts
      // System.out.println("set left zero");
      m_setPoint_Right_Voltage = 0;
    } else if ((desiredVoltage > 1) && (m_rightPosition > Constants.CLIMBER.MAX_HEIGHT)) {
      // asking to travel up, but already at max height
      System.out.println("right climber max height reached");
      m_setPoint_Right_Voltage = 0;
    } else if ((desiredVoltage < -1) && (m_rightPosition <= 0)) {
      // asking to travel down, but already at min height
      //    resetMode active needs negative values to respool climber in the pits
      //    because when you turn robot on position will be zero.
      //    So do not limit at zero.
      if (m_pitMode) {
        // go slower
        m_setPoint_Right_Voltage = desiredVoltage / 2;
      } else {
        System.out.println("right climber start height 0 reached");
        m_setPoint_Right_Voltage = 0;
      }
    } else {
      m_setPoint_Right_Voltage = m_pitMode ? desiredVoltage / 2 : desiredVoltage;
    }

    // System.out.println("right climber setSpeedVout: " + m_setPoint_Right_Voltage);
    Logger.recordOutput("Climber/RightPos", m_rightPosition);
    Logger.recordOutput("Climber/RightVolt", m_setPoint_Right_Voltage);
    m_climberRightMotor.setControl(m_voltageOut.withOutput(m_setPoint_Right_Voltage));
  }

  public void setPitMode(boolean pitMode) {
    m_pitMode = pitMode;
    Logger.recordOutput("Climber/pitMode", m_pitMode);
  }

  public boolean getPitMode() {
    return m_pitMode;
  }

  public double getLeftSpeed() {
    return this.m_setPoint_Left_Speed;
  }

  public double getRightSpeed() {
    return this.m_setPoint_Right_Speed;
  }

  public double getLeftPosition() {
    return this.m_leftPosition;
  }

  public double getRightPosition() {
    return this.m_rightPosition;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Climber");
    builder.setActuator(true);
    // builder.setSafeState(() -> setState(State.BREAK));
    // builder.addDoubleProperty("setpoint speed", null,this::setSpeed);
    builder.addDoubleProperty("left speed", this::getLeftSpeed, null);
    builder.addDoubleProperty("right speed", this::getRightSpeed, null);
    builder.addDoubleProperty("left position", this::getLeftPosition, null);
    builder.addDoubleProperty("right position", this::getRightPosition, null);
    builder.addBooleanProperty("pit mode", this::getPitMode, this::setPitMode);
  }
}
