package frc.robot.subsystems.climber;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public class ClimberIOTalonFX implements ClimberIO {
  public static final double reduction = 125 * (32 / 12);
  private final GravityTypeValue gravityType = GravityTypeValue.Arm_Cosine;

  // Hardware
  private final TalonFX talon;

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();

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
  // private final MotionMagicTorqueCurrentFOC mmtPosition = new MotionMagicTorqueCurrentFOC(0);
  private final MotionMagicVoltage mmvPosition = new MotionMagicVoltage(0);
  private final NeutralOut neutralOut = new NeutralOut();

  public ClimberIOTalonFX() {
    talon = new TalonFX(40, "rio");

    // Configure motor
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Current limits
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 70;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;
    // Feedback
    config.Feedback.RotorToSensorRatio = 1;
    config.Feedback.SensorToMechanismRatio = reduction;
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
                50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent));
    tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(4.0, tempCelsius));
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(talon));
    PhoenixUtil.registerSignals(
        false, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
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
