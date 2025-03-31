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
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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

  private double reduction;

  public RollerSystemIOTalonFX(
      int id,
      String bus,
      int currentLimitAmps,
      boolean invert,
      int followerID,
      boolean followOpposite,
      int canCoderID,
      boolean brake,
      double reduction) {

    talon = new TalonFX(id, bus);

    if (followerID != 0) {
      // this is a follower motor.  Assign its lead motor
      // also, do not need to set config.
      talon.setPosition(0);
      talon.setControl(new Follower(followerID, followOpposite));
      System.out.println("****** follower motor on elevator setup ******");
    } else {
      config.Voltage.PeakForwardVoltage = 12;
      config.Voltage.PeakReverseVoltage = -12;

      if (canCoderID != 0) {
        // this is a CANcoder.  fuse it to motor
        config.Feedback.FeedbackRemoteSensorID = canCoderID;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.RotorToSensorRatio = reduction;
        config.Feedback.SensorToMechanismRatio = 1;
        reduction = 1;
      }
      this.reduction = reduction;

      config.MotorOutput.Inverted =
          invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
      config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
      config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;

      config.CurrentLimits.StatorCurrentLimit = currentLimitAmps;
      config.CurrentLimits.StatorCurrentLimitEnable = true;

      config.TorqueCurrent.PeakForwardTorqueCurrent = currentLimitAmps;
      config.TorqueCurrent.PeakReverseTorqueCurrent = -currentLimitAmps;
      tryUntilOk(5, () -> talon.getConfigurator().apply(config));
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
                50.0, position, velocity, appliedVoltage, torqueCurrent));
    tryUntilOk(
        5, () -> BaseStatusSignal.setUpdateFrequencyForAll(250.0, supplyCurrent, tempCelsius));
    tryUntilOk(5, () -> talon.optimizeBusUtilization(0, 1.0));
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius)
            .isOK();
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble()) / reduction;
    inputs.positionRotations = position.getValueAsDouble();
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / reduction;
    inputs.velocityRotsPerSec = velocity.getValueAsDouble() / reduction;
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
  public void setPosition(double rotations, double feedforward, int slot) {
    talon.setControl(
        mmvPosition.withPosition(rotations).withFeedForward(feedforward).withSlot(slot));
    // System.out.println("\tFF set to: " + feedforward);
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
  public void setPID(Slot0Configs config0, Slot1Configs config1) {
    // slot0
    config.Slot0.kP = config0.kP;
    config.Slot0.kI = config0.kI;
    config.Slot0.kD = config0.kD;
    config.Slot0.kS = config0.kS;
    config.Slot0.kV = config0.kV;
    config.Slot0.kA = config0.kA;
    // slot1
    if (config1 != null) {
      config.Slot1.kP = config1.kP;
      config.Slot1.kI = config1.kI;
      config.Slot1.kD = config1.kD;
      config.Slot1.kS = config1.kS;
      config.Slot1.kV = config1.kV;
      config.Slot1.kA = config1.kA;
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
