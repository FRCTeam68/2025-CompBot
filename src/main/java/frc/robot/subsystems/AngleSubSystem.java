package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class AngleSubSystem extends SubsystemBase {

  public enum State {
    SPEAKER,
    SPEAKER_1M,
    AMP,
    TRAP,
    INTAKE,
    FEEDSTATION,
    SPEAKER_PODIUM,
    BRAKE,
    SPEAKER_PODIUM_SOURCE,
    CUSTOM_ANGLE
  }

  public enum Mode {
    MMV,
    MMV_FOC,
    VOUT
  }

  private State m_presentState;
  private Mode m_presentMode;

  private double m_setPoint_Position;
  private double m_setPoint_Adjust;
  private double m_speaker_position;
  private double m_speaker_1m_position;
  private double m_speaker_podium_position;
  private double m_amp_position;
  private double m_trap_position;
  private double m_intake_position;
  private double m_custom_position;

  private TalonFX m_angleLeftMotor;
  private TalonFX m_angleRightMotor;
  private MotionMagicVoltage m_angleMotorMMV;
  private NeutralOut m_brake;
  private Timer m_bumpTimer;
  private double m_bumpCount;
  private LinearFilter m_atAngleFilter;
  private double m_filteredPosition;

  public AngleSubSystem() {
    m_presentState = State.SPEAKER;
    m_presentMode = Mode.MMV;
    m_speaker_position = Constants.ANGLE.SPEAKER;
    m_speaker_1m_position = Constants.ANGLE.SPEAKER_1M;
    m_speaker_podium_position = Constants.ANGLE.SPEAKER_PODIUM;
    m_amp_position = Constants.ANGLE.AMP;
    m_trap_position = Constants.ANGLE.TRAP;
    m_intake_position = Constants.ANGLE.INTAKE;
    m_setPoint_Position = Constants.ANGLE.SPEAKER;
    m_custom_position = Constants.ANGLE.SPEAKER;
    ;
    m_setPoint_Adjust = 0;
    m_bumpTimer = new Timer();
    m_bumpTimer.start();
    m_bumpCount = 0;
    m_atAngleFilter = LinearFilter.movingAverage(5);
    m_filteredPosition = 0;

    angleMotorInit();

    System.out.println("Angle subsystem created.    mode: " + m_presentMode.toString());
    Logger.recordOutput("Angle/Comment", "Angle subsystem created");
  }

  private void angleMotorInit() {
    m_angleLeftMotor = new TalonFX(Constants.ANGLE.LEFT_CANID, "rio");
    m_angleRightMotor = new TalonFX(Constants.ANGLE.RIGHT_CANID, "rio");
    // m_angleRightMotor.setControl(new Follower(Constants.ANGLE.LEFT_CANID, true));

    m_angleMotorMMV = new MotionMagicVoltage(Constants.ANGLE.SPEAKER);

    /* Keep a neutral out so we can disable the motor */
    m_brake = new NeutralOut();

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    /* Configure current limits */
    cfg.MotionMagic.MotionMagicCruiseVelocity = 20; // 80; //106; // 5 rotations per second cruise
    cfg.MotionMagic.MotionMagicAcceleration =
        40; // 100; // Take approximately 0.5 seconds to reach max vel
    cfg.MotionMagic.MotionMagicJerk = 400; // 700;

    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // m_angleLeftMotor.setInverted(true);

    cfg.Slot0.kP = 4.8F; // 55.0F;
    cfg.Slot0.kI = 0.0F;
    cfg.Slot0.kD = 0.0F;
    cfg.Slot0.kV = 0.1; // 0.0F;
    cfg.Slot0.kS = 0.25F; // Approximately 0.25V to get the mechanism moving

    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    cfg.CurrentLimits.StatorCurrentLimit = 60;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_angleLeftMotor.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure left angle motor. Error: " + status.toString());
    }

    // ------------------------------------------
    TalonFXConfiguration cfgRight = new TalonFXConfiguration();
    /* Configure current limits */
    cfgRight.MotionMagic.MotionMagicCruiseVelocity = 80; // 106; // 5 rotations per second cruise
    cfgRight.MotionMagic.MotionMagicAcceleration =
        100; // Take approximately 0.5 seconds to reach max vel

    cfgRight.Slot0.kP = 55.0F;
    cfgRight.Slot0.kI = 0.0F;
    cfgRight.Slot0.kD = 0.0F;
    cfgRight.Slot0.kV = 0.0F;
    cfgRight.Slot0.kS = 0.25F; // Approximately 0.25V to get the mechanism moving

    cfgRight.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfgRight.CurrentLimits.SupplyCurrentLimit = 30.0;
    cfgRight.CurrentLimits.StatorCurrentLimit = 60;

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_angleRightMotor.getConfigurator().apply(cfgRight);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure right angle motor. Error: " + status.toString());
    }

    m_angleRightMotor.setControl(new Follower(Constants.ANGLE.LEFT_CANID, true));

    m_angleMotorMMV.OverrideBrakeDurNeutral = true;

    // m_angleMotor.setSafetyEnabled(false);

    zeroAngleSensor();
  }

  public void zeroAngleSensor() {
    m_angleLeftMotor.setVoltage(0);
    // m_angleLeftMotor.setControl(m_brake);
    m_angleLeftMotor.setPosition(0);
    // m_angleRightMotor.setPosition(0);
    m_setPoint_Position = Constants.ANGLE.MIN_POSITION; // 0
    System.out.println("zeroAngleSensor");
  }

  // public void setPositionJoy(double desiredAjustPosition){
  //     m_bumpCount = m_bumpCount + 1;
  //     if ((Math.abs(desiredAjustPosition)>0.5) && (m_bumpTimer.hasElapsed(1))) {
  //         m_bumpTimer.restart();

  //         m_setPoint_Adjust = m_setPoint_Adjust +
  // desiredAjustPosition*Constants.ANGLE.BUMP_VALUE;
  //         System.out.println("desired: " + desiredAjustPosition
  //                            + "setpoint: " + m_setPoint_Position
  //                            + ", plus: " + m_setPoint_Adjust
  //                            + ", count: " + m_bumpCount);
  //         setPosition(m_setPoint_Position + m_setPoint_Adjust);
  //     }
  // }

  public void bumpPosition(double bumpAmount) {
    double new_value = m_setPoint_Position + bumpAmount;
    // if (Math.abs(new_value) < 1){
    //     new_value = bumpAmount < 0? -1 : 1;
    // }
    setPosition(new_value);
  }

  public void setPosition(double desiredPosition) {

    // System.out.println("  set angle desired position: " + desiredPosition);
    if (desiredPosition < Constants.ANGLE.MIN_POSITION) {
      m_setPoint_Position = Constants.ANGLE.MIN_POSITION;
      System.out.println("  trimmed to min position: " + Constants.ANGLE.MIN_POSITION);
      Logger.recordOutput(
          "Angle/Comment", "trimmed to min position: " + Constants.ANGLE.MIN_POSITION);
    } else if (desiredPosition > Constants.ANGLE.MAX_POSITION) {
      m_setPoint_Position = Constants.ANGLE.MAX_POSITION;
      System.out.println("  trimmed to max position: " + Constants.ANGLE.MAX_POSITION);
      Logger.recordOutput(
          "Angle/Comment", "trimmed to max position: " + Constants.ANGLE.MAX_POSITION);
    } else {
      m_setPoint_Position = desiredPosition;
    }
    Logger.recordOutput("Angle/setPosition", desiredPosition);

    switch (m_presentMode) {
      default:
      case MMV:
        m_angleLeftMotor.setControl(m_angleMotorMMV.withPosition(m_setPoint_Position));
        break;

      case MMV_FOC:
        // TBD
        break;
    }
  }

  // this is the state machine of the notesubsystem
  @Override
  public void periodic() {
    double motorPosition = m_angleLeftMotor.getPosition().getValueAsDouble();
    m_filteredPosition = m_atAngleFilter.calculate(motorPosition);
    Logger.recordOutput("Angle/AtAngle", motorPosition);
    Logger.recordOutput("Angle/AtAngleF", m_filteredPosition);
  }

  public boolean atAngle() {
    boolean conditionMet = Math.abs(m_setPoint_Position - m_filteredPosition) < 1.0;
    // conditionMet = true;  //for simulation since position always comes back 0
    return conditionMet;
  }

  public double getAngleF() {
    return m_filteredPosition;
  }

  public void setSpeakerPosition(double desiredPosition) {
    m_speaker_position = desiredPosition;
  }

  public void setAmpPosition(double desiredPosition) {
    m_amp_position = desiredPosition;
  }

  public void setTrapPosition(double desiredPosition) {
    m_trap_position = desiredPosition;
  }

  public void setIntakePosition(double desiredPosition) {
    m_intake_position = desiredPosition;
  }

  public void setCustomPosition(double desiredPosition) {
    m_custom_position = desiredPosition;
  }

  public double getPosition() {
    return this.m_setPoint_Position;
  }

  public double getSpeakerPosition() {
    return this.m_speaker_position;
  }

  public double getAmpPosition() {
    return this.m_amp_position;
  }

  public double getTrapPosition() {
    return this.m_trap_position;
  }

  public double getIntakePosition() {
    return this.m_intake_position;
  }

  public double getCustomPosition() {
    return this.m_custom_position;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Angle");
    builder.setActuator(true);
    // builder.setSafeState(() -> setState(State.BREAK));
    builder.addDoubleProperty("setpoint position", this::getPosition, this::setPosition);
    builder.addDoubleProperty(
        "speaker position", this::getSpeakerPosition, this::setSpeakerPosition);
    builder.addDoubleProperty("amp position", this::getAmpPosition, this::setAmpPosition);
    builder.addDoubleProperty("trap position", this::getTrapPosition, this::setTrapPosition);
    builder.addDoubleProperty("intake position", this::getIntakePosition, this::setIntakePosition);
    builder.addDoubleProperty("custom position", this::getCustomPosition, this::setCustomPosition);
    builder.addStringProperty("State", () -> m_presentState.toString(), null);
    builder.addStringProperty("Mode", () -> m_presentMode.toString(), null);
  }

  public void setState(State wantedState) {
    m_presentState = wantedState;

    double desiredPosition = 0;

    // System.out.println("set angle state: " + m_presentState.toString());
    Logger.recordOutput("Angle/State", m_presentState);

    switch (m_presentState) {
      default:
      case SPEAKER:
        desiredPosition = m_speaker_position;
        break;
      case SPEAKER_1M:
        desiredPosition = m_speaker_1m_position;
        break;
      case SPEAKER_PODIUM:
        desiredPosition = m_speaker_podium_position;
        break;
      case SPEAKER_PODIUM_SOURCE:
        desiredPosition = Constants.ANGLE.SPEAKER_PODIUM_SOURCE;
        break;
      case AMP:
        desiredPosition = m_amp_position;
        ;
        break;
      case TRAP:
        desiredPosition = m_trap_position;
        break;
      case INTAKE:
        desiredPosition = m_intake_position;
        break;
      case FEEDSTATION:
      case BRAKE:
        // m_angleLeftMotor.setControl(m_brake);
        m_angleLeftMotor.setVoltage(0);
        break;
      case CUSTOM_ANGLE:
        desiredPosition = m_custom_position;
        break;
    }
    m_setPoint_Adjust = 0;

    setPosition(desiredPosition);
  }

  public State getState() {
    return this.m_presentState;
  }
}
