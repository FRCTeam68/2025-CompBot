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
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterSubSystem extends SubsystemBase {

  // public enum State{
  //     IDLE,
  //     SPINUP,
  //     SHOOT,
  //     STOP
  // }

  public enum Mode {
    VOLTAGE_OUT,
    VOLTAGE_FOC,
    CURRENTTORQUE_FOC
  }

  // private State m_presentState;
  private Mode m_presentMode;
  private double m_setPoint_Left_Speed;
  private double m_setPoint_Right_Speed;
  private LoggedNetworkNumber m_spinUp_Speed =
      new LoggedNetworkNumber("/Tuning/Shooter/spinup_speed", 0.0);
  private double m_rightOffset_Speed;
  private TalonFX m_shooterLeftMotor;
  private TalonFX m_shooterRightMotor;
  private VoltageOut m_voltageOut;
  private VelocityVoltage m_voltageVelocity;
  private VelocityTorqueCurrentFOC m_torqueVelocity;
  private NeutralOut m_coast;
  private double m_setPoint_Left_Voltage;
  private double m_setPoint_Right_Voltage;
  private double m_rightOffset_Voltage;
  private LinearFilter m_atSpeedFilter;
  private double m_filteredLeftSpeed;
  private boolean m_stop;

  // left is bottom
  // right is top
  public ShooterSubSystem() {
    // m_presentState = State.IDLE;
    m_presentMode = Mode.VOLTAGE_FOC;
    m_setPoint_Left_Speed = 0;
    m_setPoint_Right_Speed = 0;
    m_rightOffset_Speed = Constants.SHOOTER.RIGHT_OFFSET;
    m_atSpeedFilter = LinearFilter.movingAverage(5);
    m_filteredLeftSpeed = 0;
    m_stop = true;

    shooterMotorsInit();

    // // Shuffleboard.getTab("shoot Tuning").add(new InstantCommand(this::setTuningSpeed));
    // ShuffleboardTab tab =Shuffleboard.getTab("shooter333");
    // BooleanTopic shooterTuningEnable = tab.add("Enable shooter tune",false).getEntry();
    // new NetworkButton(shooterTuningEnable).ontrue(new InstantCommand(this::setTuningSpeed));
  }

  private void shooterMotorsInit() {
    m_shooterLeftMotor = new TalonFX(Constants.SHOOTER.LEFT_CANID); // bottom
    m_shooterRightMotor = new TalonFX(Constants.SHOOTER.RIGHT_CANID); // top

    m_voltageOut = new VoltageOut(0);

    /* Start at velocity 0, enable FOC, no feed forward, use slot 0 */
    m_voltageVelocity = new VelocityVoltage(0);
    /* Start at velocity 0, no feed forward, use slot 1 */
    m_torqueVelocity = new VelocityTorqueCurrentFOC(0);
    /* Keep a neutral out so we can disable the motor */
    m_coast = new NeutralOut();

    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    // 0.11, 0.5, 0.0001, 0.12.  , .21 got there faster,  1 was too high (oscilated around zero when
    // off)
    configs.Slot0.kP = .2; // 0.11 An error of 1 rotation per second results in 2V output
    configs.Slot0.kI =
        0; //  0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD =
        0; //  0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV =
        0.13; // 0.13; // 12V/89rps = 0.134volts;  0.03volts minimum to make wheels turn
    // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation
    // per second

    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
    configs.Slot1.kI =
        0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    configs.Slot1.kD =
        0.001; // A change of 1000 rotation per second squared results in 1 amp output

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 60;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -60;

    configs.CurrentLimits.StatorCurrentLimit = 60;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;

    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_shooterLeftMotor.getConfigurator().apply(configs); // bottom
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println(
          "Could not apply configs to left shooter motor, error code: " + status.toString());
    }

    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    /* Retry config apply up to 5 times, report if failure */
    for (int i = 0; i < 5; ++i) {
      status = m_shooterRightMotor.getConfigurator().apply(configs); // top
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println(
          "Could not apply configs to right shooter motor, error code: " + status.toString());
    }

    System.out.println("shooter subsystem created.   " + "mode: " + m_presentMode.toString());
  }

  public void configureVelocityPID(double kp, double ki, double kd, double kv) {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.Slot0.kP = kp;
    configs.Slot0.kI = ki;
    configs.Slot0.kD = kd;
    configs.Slot0.kV = kv;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_shooterLeftMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println(
          "Could not apply configs to left shooter motor, error code: " + status.toString());
    }

    /* Retry config apply up to 5 times, report if failure */
    for (int i = 0; i < 5; ++i) {
      status = m_shooterRightMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println(
          "Could not apply configs to right shooter motor, error code: " + status.toString());
    }

    System.out.println("shooter pid assigned");
  }

  public void setSpinUpSpeed(double desiredSpeed) {
    m_spinUp_Speed.set(desiredSpeed);
  }

  public void setRightOffsetSpeed(double desiredSpeed) {
    m_rightOffset_Speed = desiredSpeed;
  }

  public void setTuningSpeed() {
    setSpeed(getSpinUpSpeed());
  }

  public void setSpeedVout(double desiredVoltage) {
    // System.out.println("shooter setSpeedesired" + desiredVoltage);
    if (m_presentMode == Mode.VOLTAGE_OUT) {
      if (Math.abs(desiredVoltage) <= .1) { // Joystick deadzone
        // System.out.println("set zeros");
        m_setPoint_Left_Voltage = 0;
        m_setPoint_Right_Voltage = 0;
      } else {
        m_setPoint_Left_Voltage = desiredVoltage;
        m_setPoint_Right_Voltage = -desiredVoltage + m_rightOffset_Voltage;
      }
      // System.out.println("  shooter setSpeedVout, L: " + m_setPoint_Left_Voltage
      //                                      + " , R: " + m_setPoint_Right_Voltage);
      Logger.recordOutput("Shooter/setLeftVoltage", m_setPoint_Left_Voltage);
      Logger.recordOutput("Shooter/setRightVoltage", m_setPoint_Right_Voltage);
      m_shooterLeftMotor.setControl(m_voltageOut.withOutput(m_setPoint_Left_Voltage));
      m_shooterRightMotor.setControl(m_voltageOut.withOutput(m_setPoint_Right_Voltage));
    }
  }

  public void bumpSpeed(double bumpAmount) {
    double new_value = m_setPoint_Left_Speed + bumpAmount;
    if (Math.abs(new_value) < 1) {
      new_value = bumpAmount < 0 ? -1 : 1;
    }
    setSpeed(new_value);
  }

  public void setSpeed(double desiredRotationsPerSecond) {

    // System.out.println("  set Shooter desired speed: " + desiredRotationsPerSecond);
    Logger.recordOutput("Shooter/setDesiredSpeed", desiredRotationsPerSecond);

    if (Math.abs(desiredRotationsPerSecond) < 1) { // Joystick deadzone
      desiredRotationsPerSecond = 0;
      m_setPoint_Left_Speed = 0;
      m_setPoint_Right_Speed = 0;
      m_shooterLeftMotor.setControl(m_coast);
      m_shooterRightMotor.setControl(m_coast);
      Logger.recordOutput("Shooter/setLeftSpeed", m_setPoint_Left_Speed);
      Logger.recordOutput("Shooter/setRightSpeed", m_setPoint_Right_Speed);
    } else {
      m_stop = false;

      if (desiredRotationsPerSecond > Constants.SHOOTER.MAX_SPEED) {
        m_setPoint_Left_Speed = Constants.SHOOTER.MAX_SPEED;
        m_setPoint_Right_Speed = -Constants.SHOOTER.MAX_SPEED;
        // System.out.println("  trimmed to max speed: " + Constants.SHOOTER.MAX_SPEED);
        Logger.recordOutput(
            "Shooter/Comment", "trimmed to max speed: " + Constants.SHOOTER.MAX_SPEED);
      } else {
        m_setPoint_Left_Speed = desiredRotationsPerSecond;
        m_setPoint_Right_Speed = -desiredRotationsPerSecond + m_rightOffset_Speed; // top
      }

      Logger.recordOutput("Shooter/setLeftSpeed", m_setPoint_Left_Speed);
      Logger.recordOutput("Shooter/setRightSpeed", m_setPoint_Right_Speed);
      switch (m_presentMode) {
        case VOLTAGE_OUT:
          m_setPoint_Left_Voltage =
              (m_setPoint_Left_Speed / Constants.SHOOTER.MAX_SPEED) * Constants.ROLLER.MAX_VOLTAGE;
          m_setPoint_Right_Voltage =
              (m_setPoint_Right_Speed / Constants.SHOOTER.MAX_SPEED) * Constants.ROLLER.MAX_VOLTAGE;
          // System.out.println("         desired Voltage, L: " + m_setPoint_Left_Voltage
          //                                         + " , R: " + m_setPoint_Right_Voltage);
          m_shooterLeftMotor.setControl(m_voltageOut.withOutput(m_setPoint_Left_Voltage));
          m_shooterRightMotor.setControl(m_voltageOut.withOutput(m_setPoint_Right_Voltage));
          break;
        default:
        case VOLTAGE_FOC:
          /* Use voltage velocity */
          m_shooterLeftMotor.setControl(m_voltageVelocity.withVelocity(m_setPoint_Left_Speed));
          m_shooterRightMotor.setControl(m_voltageVelocity.withVelocity(m_setPoint_Right_Speed));
          break;

        case CURRENTTORQUE_FOC:
          double friction_torque =
              (desiredRotationsPerSecond > 0)
                  ? 1
                  : -1; // To account for friction, we add this to the arbitrary feed forward
          /* Use torque velocity */
          m_shooterLeftMotor.setControl(
              m_torqueVelocity
                  .withVelocity(m_setPoint_Left_Speed)
                  .withFeedForward(friction_torque));
          m_shooterRightMotor.setControl(
              m_torqueVelocity
                  .withVelocity(m_setPoint_Right_Speed)
                  .withFeedForward(friction_torque));
          break;
      }
    }
  }

  // this is the state machine of the notesubsystem
  @Override
  public void periodic() {
    double motorLeftSpeed = m_shooterLeftMotor.getVelocity().getValueAsDouble();
    double motorRightSpeed = m_shooterRightMotor.getVelocity().getValueAsDouble(); // top
    m_filteredLeftSpeed = m_atSpeedFilter.calculate(motorLeftSpeed);
    Logger.recordOutput("Speed/LeftSpeed", motorLeftSpeed);
    Logger.recordOutput("Speed/RightSpeed", motorRightSpeed);
    Logger.recordOutput("Speed/LeftSpeedF", m_filteredLeftSpeed);
    Logger.recordOutput("Speed/LeftVoltage", m_shooterLeftMotor.get());
    Logger.recordOutput("Speed/RightVoltage", m_shooterRightMotor.get());

    if ((m_setPoint_Left_Speed == 0) && (!m_stop)) {
      if (m_filteredLeftSpeed < Constants.SHOOTER.SPINLOW_SPEED) {
        // we set speed to 0 to coast down shooter.
        // now we are below speed we want to stay at, so set speed now
        setRightOffsetSpeed(0);
        setSpeed(Constants.SHOOTER.SPINLOW_SPEED);
      }
    }
  }

  public boolean atSpeed() {
    boolean conditionMet = Math.abs(m_setPoint_Left_Speed - m_filteredLeftSpeed) < 3.0;
    // conditionMet = true;  //for simulation since position always comes back 0
    return conditionMet;
  }

  public double getSpinUpSpeed() {
    return this.m_spinUp_Speed.get();
  }

  public double getRightOffsetSpeed() {
    return this.m_rightOffset_Speed;
  }

  public double getLeftSpeed() {
    return this.m_setPoint_Left_Speed;
  }

  public double getRightSpeed() {
    return this.m_setPoint_Right_Speed;
  }

  public double getSpeed() {
    return this.m_setPoint_Left_Speed;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shooter");
    builder.setActuator(true);
    // builder.setSafeState(() -> setState(State.BREAK));
    // builder.addDoubleProperty("setpoint speed", null,this::setSpeed);
    builder.addDoubleProperty("left speed", this::getLeftSpeed, null);
    builder.addDoubleProperty("right speed", this::getRightSpeed, null);
    builder.addDoubleProperty("spinup speed", this::getSpinUpSpeed, this::setSpinUpSpeed);
    builder.addDoubleProperty(
        "right offset speed", this::getRightOffsetSpeed, this::setRightOffsetSpeed);
    // builder.addStringProperty("State", () -> m_presentState.toString(),null);
    builder.addStringProperty("Mode", () -> m_presentMode.toString(), null);
  }

  public void stop() {
    m_stop = true;
  }

  // public void setState(State wantedState) {
  // 	// m_presentState = wantedState;

  //     double desiredSpeed = 0;

  //     // System.out.println("set shooter state: " + wantedState.toString());
  //     Logger.recordOutput("Shooter/setState",  wantedState);

  //     switch(wantedState){

  //         case SPINUP:   // L1
  //             desiredSpeed = m_spinUp_Speed;
  //             m_stop = false;
  //             break;
  //         case SHOOT:     // L2
  //             // TBD
  //             // check if note present (top beam break is true)
  //             // check if shooter wheels are atspeed,
  //             // finally spin TopRoller at shoot speed, to feed note into shooter
  //             // wait 2 seconds, set speed back to 0
  //             break;
  //         case STOP:
  //             desiredSpeed = 0;
  //             m_stop = true;
  //         default:
  //         case IDLE:
  //             //nothing, stay with zero speed
  //             break;
  //     }

  //     this.setSpeed(desiredSpeed);
  // }

  // public State getState(){
  //     return this.m_presentState;
  // }

}
