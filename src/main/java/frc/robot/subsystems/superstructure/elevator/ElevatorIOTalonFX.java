package frc.robot.subsystems.superstructure.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
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
  private final StatusSignal<Angle> followerPosition;
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
    followerPosition = followerTalon.getPosition();
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
                followerPosition,
                velocity,
                appliedVoltage,
                torqueCurrent,
                followerAppliedVoltage,
                followerTorqueCurrent));
    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                4, supplyCurrent, tempCelsius, followerSupplyCurrent, followerTempCelsius));
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(talon, followerTalon));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.connected =
        connectedDebouncer.calculate(
            BaseStatusSignal.refreshAll(
                    position, followerPosition, velocity, appliedVoltage, torqueCurrent)
                .isOK());
    inputs.position = position.getValueAsDouble();
    inputs.velocityRotsPerSec = velocity.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
    inputs.followerConnected =
        followerConnectedDebouncer.calculate(
            BaseStatusSignal.refreshAll(followerAppliedVoltage, followerTorqueCurrent).isOK());
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
    if (newconfig.length > 1) {
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
    if (newconfig.length > 2) {
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
