// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public class RollerSystemIOTalonFX implements RollerSystemIO {
  private final TalonFX talon;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final VelocityVoltage voltageVelocity = new VelocityVoltage(0);
  ;
  // private final VelocityTorqueCurrentFOC torqueVelocity  = new VelocityTorqueCurrentFOC(0);
  private final MotionMagicVoltage mmvPosition = new MotionMagicVoltage(0);
  private final NeutralOut neutralOut = new NeutralOut();

  private final double reduction;

  public RollerSystemIOTalonFX(
      int id,
      String bus,
      int currentLimitAmps,
      boolean invert,
      int followerID,
      boolean followOpposite,
      boolean brake,
      double reduction,
      Slot0Configs slot0Configs) {
    this.reduction = reduction;
    talon = new TalonFX(id, bus);

    config.MotionMagic.MotionMagicCruiseVelocity =
        20; // 80; //106; // 5 rotations per second cruise
    config.MotionMagic.MotionMagicAcceleration =
        40; // 100; // Take approximately 0.5 seconds to reach max vel
    config.MotionMagic.MotionMagicJerk = 400; // 700;
    config.Slot0.kP = slot0Configs.kP;
    config.Slot0.kI = slot0Configs.kI;
    config.Slot0.kD = slot0Configs.kD;
    config.Slot0.kS = slot0Configs.kS;
    config.Slot0.kV = slot0Configs.kV;
    config.Slot0.kV = slot0Configs.kA;

    config.Voltage.PeakForwardVoltage = 12;
    config.Voltage.PeakReverseVoltage = -12;

    config.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.TorqueCurrent.PeakForwardTorqueCurrent = currentLimitAmps;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -currentLimitAmps;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config));

    if (followerID != 0) {
      talon.setControl(new Follower(followerID, followOpposite));
    }

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
                50.0,
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius));
    tryUntilOk(5, () -> talon.optimizeBusUtilization(0, 1.0));
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius)
            .isOK();
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble()) / reduction;
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / reduction;
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
  public void setSpeed(double speed) {
    talon.setControl(voltageVelocity.withVelocity(speed));
  }

  @Override
  public void setPosition(double rotations) {
    talon.setControl(mmvPosition.withPosition(rotations));
  }

  @Override
  public void setZero() {
    setVolts(0);
    // m_angleLeftMotor.setControl(m_brake);
    setPosition(0);
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void setPID(Slot0Configs newconfig) {
    config.Slot0.kP = newconfig.kP;
    config.Slot0.kI = newconfig.kI;
    config.Slot0.kD = newconfig.kD;
    config.Slot0.kS = newconfig.kS;
    config.Slot0.kV = newconfig.kV;
    config.Slot0.kA = newconfig.kA;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
  }
}
