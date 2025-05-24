package frc.robot.subsystems.superstructure.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
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
import lombok.Getter;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public class ElevatorIOTalonFX implements ElevatorIO {
  @Getter private static final double reduction = 5;
  @Getter private static final double spoolDiameter = 1.751; // inches
  private final GravityTypeValue gravityType = GravityTypeValue.Elevator_Static;

  // Hardware
  private final TalonFX talon;
  private final TalonFX followerTalon;

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;
  private final StatusSignal<Voltage> followerAppliedVoltage;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Current> followerTorqueCurrent;
  private final StatusSignal<Temperature> followerTempCelsius;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);
  private final Debouncer followerConnectedDebouncer = new Debouncer(0.5);

  // Control requests
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);

  @SuppressWarnings("unused")
  private final MotionMagicTorqueCurrentFOC mmtPosition =
      new MotionMagicTorqueCurrentFOC(0).withUpdateFreqHz(0);

  private final MotionMagicVoltage mmvPosition = new MotionMagicVoltage(0);
  private final NeutralOut neutralOut = new NeutralOut();

  public ElevatorIOTalonFX() {
    talon = new TalonFX(33, "rio");
    followerTalon = new TalonFX(32, "rio");
    followerTalon.setControl(new Follower(talon.getDeviceID(), true));

    // Configure Motor
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5.4;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    // Feedback
    config.Feedback.SensorToMechanismRatio = reduction;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    // Configure status signals
    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();
    followerAppliedVoltage = followerTalon.getMotorVoltage();
    followerSupplyCurrent = followerTalon.getSupplyCurrent();
    followerTorqueCurrent = followerTalon.getTorqueCurrent();
    followerTempCelsius = followerTalon.getDeviceTemp();

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
                followerAppliedVoltage,
                followerSupplyCurrent,
                followerTorqueCurrent));
    tryUntilOk(
        5, () -> BaseStatusSignal.setUpdateFrequencyForAll(4, tempCelsius, followerTempCelsius));
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(talon, followerTalon));
    PhoenixUtil.registerSignals(
        false,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius,
        followerAppliedVoltage,
        followerSupplyCurrent,
        followerTorqueCurrent,
        followerTempCelsius);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.connected =
        connectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                position, velocity, appliedVoltage, supplyCurrent, torqueCurrent));
    inputs.position = position.getValueAsDouble();
    inputs.velocityRotsPerSec = velocity.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
    inputs.followerConnected =
        followerConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                followerAppliedVoltage, followerSupplyCurrent, followerTorqueCurrent));
    inputs.followerAppliedVoltage = followerAppliedVoltage.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupplyCurrent.getValueAsDouble();
    inputs.followerTorqueCurrentAmps = followerTorqueCurrent.getValueAsDouble();
    inputs.followerTempCelsius = followerTempCelsius.getValueAsDouble();
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
  public void setPID(SlotConfigs... newConfig) {
    for (int i = 0; i < newConfig.length; i++) {
      SlotConfigs slotConfig =
          new SlotConfigs()
              .withGravityType(gravityType)
              .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
              .withKP(newConfig[i].kP)
              .withKI(newConfig[i].kI)
              .withKD(newConfig[i].kD)
              .withKV(newConfig[i].kV)
              .withKA(newConfig[i].kA)
              .withKG(newConfig[i].kG)
              .withKS(newConfig[i].kS);

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

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void zero() {
    setVolts(0);
    talon.setPosition(0);
  }
}
