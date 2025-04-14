package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.WRIST;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public class WristIOTalonFX implements WristIO {
  public static final double reduction = WRIST.REDUCTION;
  private final GravityTypeValue gravityType = GravityTypeValue.Arm_Cosine;

  // Hardware
  private final TalonFX talon;
  private final CANcoder cancoder;

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

  // Status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  // Control requests
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final MotionMagicTorqueCurrentFOC mmtPosition = new MotionMagicTorqueCurrentFOC(0);
  private final NeutralOut neutralOut = new NeutralOut();

  public WristIOTalonFX() {
    talon = new TalonFX(WRIST.CANID, WRIST.CANBUS);
    cancoder = new CANcoder(WRIST.CANCODER_CANID, WRIST.CANBUS);

    // configure sensor
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = WRIST.CANCODER_OFFSET;
    tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25));

    // Configure Motor
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 70;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;
    config.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.RotorToSensorRatio = reduction;
    config.Feedback.SensorToMechanismRatio = 1;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    // Configure status signals
    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, position, velocity, appliedVoltage, torqueCurrent));
    tryUntilOk(
        5, () -> BaseStatusSignal.setUpdateFrequencyForAll(250.0, supplyCurrent, tempCelsius));
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(talon));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.connected =
        connectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(position, velocity, appliedVoltage, torqueCurrent));
    inputs.positionRotations = position.getValueAsDouble();
    inputs.velocityRotsPerSec = velocity.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void setVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setPosition(double position, int slot) {
    talon.setControl(mmtPosition.withPosition(position).withSlot(slot));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void zero() {
    setVolts(0);
    talon.setPosition(0);
  }

  @Override
  public void setPID(SlotConfigs... newconfig) {
    config.Slot0.GravityType = gravityType;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    config.Slot0.kP = newconfig[0].kP;
    config.Slot0.kI = newconfig[0].kI;
    config.Slot0.kD = newconfig[0].kD;
    config.Slot0.kV = newconfig[0].kV;
    config.Slot0.kA = newconfig[0].kA;
    config.Slot0.kG = newconfig[0].kG;
    config.Slot0.kS = newconfig[0].kS;
    if (newconfig.length >= 1) {
      config.Slot1.GravityType = gravityType;
      config.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
      config.Slot1.kP = newconfig[1].kP;
      config.Slot1.kI = newconfig[1].kI;
      config.Slot1.kD = newconfig[1].kD;
      config.Slot1.kV = newconfig[1].kV;
      config.Slot1.kA = newconfig[1].kA;
      config.Slot1.kG = newconfig[1].kG;
      config.Slot1.kS = newconfig[1].kS;
    }
    if (newconfig.length >= 2) {
      config.Slot2.GravityType = gravityType;
      config.Slot2.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
      config.Slot2.kP = newconfig[2].kP;
      config.Slot2.kI = newconfig[2].kI;
      config.Slot2.kD = newconfig[2].kD;
      config.Slot2.kV = newconfig[2].kV;
      config.Slot2.kA = newconfig[2].kA;
      config.Slot2.kG = newconfig[2].kG;
      config.Slot2.kS = newconfig[2].kS;
    }
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void setMotionMagic(MotionMagicConfigs newconfig) {
    config.MotionMagic.MotionMagicCruiseVelocity = newconfig.MotionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = newconfig.MotionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = newconfig.MotionMagicJerk;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
  }
}
