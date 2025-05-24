// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {
  private static final LoggedTunableNumber drivekS =
      new LoggedTunableNumber("Drive/Module/Drive/kS");
  private static final LoggedTunableNumber drivekV =
      new LoggedTunableNumber("Drive/Module/Drive/kV");
  private static final LoggedTunableNumber drivekA =
      new LoggedTunableNumber("Drive/Module/Drive/kA");
  private static final LoggedTunableNumber drivekP =
      new LoggedTunableNumber("Drive/Module/Drive/kP");
  private static final LoggedTunableNumber drivekD =
      new LoggedTunableNumber("Drive/Module/Drive/kD");
  private static final LoggedTunableNumber turnkS = new LoggedTunableNumber("Drive/Module/Turn/kS");
  private static final LoggedTunableNumber turnkV = new LoggedTunableNumber("Drive/Module/Turn/kV");
  private static final LoggedTunableNumber turnkA = new LoggedTunableNumber("Drive/Module/Turn/kA");
  private static final LoggedTunableNumber turnkP = new LoggedTunableNumber("Drive/Module/Turn/kP");
  private static final LoggedTunableNumber turnkD = new LoggedTunableNumber("Drive/Module/Turn/kD");

  static {
    // Drive defaults
    drivekS.initDefault(0.16);
    drivekV.initDefault(0.12);
    drivekA.initDefault(0.004);
    drivekP.initDefault(0.16);
    drivekD.initDefault(0);
    // Turn defaults
    turnkS.initDefault(0.02);
    turnkV.initDefault(2.6);
    turnkA.initDefault(0.1);
    turnkP.initDefault(100.0);
    turnkD.initDefault(3.6);
  }

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private final Alert driveTempAlert;
  private final Alert turnTempAlert;
  private final Alert turnEncoderSyncStickyAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    io.setDrivePID(
        new Slot0Configs()
            .withKS(drivekS.getAsDouble())
            .withKV(drivekV.getAsDouble())
            .withKA(drivekA.getAsDouble())
            .withKP(drivekP.getAsDouble())
            .withKD(drivekD.getAsDouble()));

    io.setTurnPID(
        new Slot0Configs()
            .withKS(turnkS.getAsDouble())
            .withKV(turnkV.getAsDouble())
            .withKA(turnkA.getAsDouble())
            .withKP(turnkP.getAsDouble())
            .withKD(turnkD.getAsDouble()));

    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Disconnected turn encoder on module " + Integer.toString(index) + ".",
            AlertType.kError);
    driveTempAlert =
        new Alert(
            "Drive motor over temp on module " + Integer.toString(index) + ".", AlertType.kWarning);
    turnTempAlert =
        new Alert(
            "Turn motor over temp on module " + Integer.toString(index) + ".", AlertType.kWarning);
    turnEncoderSyncStickyAlert =
        new Alert(
            "Turn motor integrated sensor and fused encoder out of sync on module "
                + Integer.toString(index)
                + ". This is likely caused by a damaged turn belt or an incorrect turn gear ratio. This alert will remain until power is cycled.",
            AlertType.kError);
  }

  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
  }

  public void periodic() {
    // Update tunable numbers
    if (Constants.tuningMode) {
      if (drivekS.hasChanged(hashCode())
          || drivekV.hasChanged(hashCode())
          || drivekA.hasChanged(hashCode())
          || drivekP.hasChanged(hashCode())
          || drivekD.hasChanged(hashCode())) {
        io.setDrivePID(
            new Slot0Configs()
                .withKS(drivekS.getAsDouble())
                .withKV(drivekV.getAsDouble())
                .withKA(drivekA.getAsDouble())
                .withKP(drivekP.getAsDouble())
                .withKD(drivekD.getAsDouble()));
      }
      if (turnkS.hasChanged(hashCode())
          || turnkV.hasChanged(hashCode())
          || turnkA.hasChanged(hashCode())
          || turnkP.hasChanged(hashCode())
          || turnkD.hasChanged(hashCode())) {
        io.setTurnPID(
            new Slot0Configs()
                .withKS(turnkS.getAsDouble())
                .withKV(turnkV.getAsDouble())
                .withKA(turnkA.getAsDouble())
                .withKP(turnkP.getAsDouble())
                .withKD(turnkD.getAsDouble()));
      }
    }

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * DriveConstants.wheelRadius;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
    driveTempAlert.set(inputs.driveTemp > 50);
    turnTempAlert.set(inputs.turnTemp > 50);
    turnEncoderSyncStickyAlert.set(inputs.turnEncoderSyncStickyFault);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    // Apply setpoints
    io.setDriveVelocity(state.speedMetersPerSecond / DriveConstants.wheelRadius);
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(new Rotation2d());
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * DriveConstants.wheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DriveConstants.wheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }

  /* Sets brake mode to {@code enabled} */
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }
}
