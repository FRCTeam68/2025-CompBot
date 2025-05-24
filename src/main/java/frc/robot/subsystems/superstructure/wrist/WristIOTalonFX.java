package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.util.PhoenixUtil;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public class WristIOTalonFX implements WristIO {
  public static final double reduction = 62.5;
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
  private final StatusSignal<MagnetHealthValue> magnetHealth;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);
  private final Debouncer cancoderConnectedDebouncer = new Debouncer(0.5);

  // Control requests
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final MotionMagicTorqueCurrentFOC mmtPosition = new MotionMagicTorqueCurrentFOC(0);
  private final MotionMagicVoltage mmvPosition = new MotionMagicVoltage(0);
  private final NeutralOut neutralOut = new NeutralOut();

  public WristIOTalonFX() {
    talon = new TalonFX(31, "rio");
    cancoder = new CANcoder(36, "rio");

    // configure sensor
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = -0.056640625;
    tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25));

    // Configure Motor
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Current limits
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 70;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;
    // Motion limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = SuperstructureConstants.WRIST.max;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = SuperstructureConstants.WRIST.min;
    // Feedback
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
    magnetHealth = cancoder.getMagnetHealth();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                magnetHealth));
    tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(4.0, tempCelsius));
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(talon, cancoder));
    PhoenixUtil.registerSignals(
        false,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius,
        magnetHealth);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.connected =
        connectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                position, velocity, appliedVoltage, supplyCurrent, torqueCurrent));
    inputs.positionRotations = position.getValueAsDouble();
    inputs.velocityRotsPerSec = velocity.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
    inputs.CANcoderConnected =
        cancoderConnectedDebouncer.calculate(BaseStatusSignal.isAllGood(magnetHealth));
    inputs.CANcoderMagnetHealth = magnetHealth.getValue();
  }

  @Override
  public void setVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setPosition(double position, int slot) {
    // talon.setControl(mmtPosition.withPosition(position).withSlot(slot));
    talon.setControl(mmvPosition.withPosition(position).withSlot(slot));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void zero() {
    setVolts(0);
    talon.setPosition(0);
    cancoder.setPosition(0);
  }

  @Override
  public void setPID(SlotConfigs... newconfig) {
    for (int i = 0; i < newconfig.length; i++) {
      SlotConfigs slotConfig =
          new SlotConfigs()
              .withGravityType(gravityType)
              .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
              .withKP(newconfig[i].kP)
              .withKI(newconfig[i].kI)
              .withKD(newconfig[i].kD)
              .withKV(newconfig[i].kV)
              .withKA(newconfig[i].kA)
              .withKG(newconfig[i].kG)
              .withKS(newconfig[i].kS);

      switch (i) {
        case 0 -> config.Slot0 = Slot0Configs.from(slotConfig);
        case 1 -> config.Slot1 = Slot1Configs.from(slotConfig);
        case 2 -> config.Slot2 = Slot2Configs.from(slotConfig);
      }
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
