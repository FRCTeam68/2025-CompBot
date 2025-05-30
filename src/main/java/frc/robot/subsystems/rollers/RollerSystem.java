// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RollerSystem extends SubsystemBase {
  private final String name;
  private final RollerSystemIO io;
  protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();
  private final Alert disconnected;
  protected final Timer stateTimer = new Timer();

  private LoggedTunableNumber rollerkP;
  private LoggedTunableNumber rollerkI;
  private LoggedTunableNumber rollerkD;
  private LoggedTunableNumber rollerkS;
  private LoggedTunableNumber rollerkV;
  private LoggedTunableNumber rollerkA;

  private LoggedTunableNumber rollerMMV;
  private LoggedTunableNumber rollerMMA;
  private LoggedTunableNumber rollerMMJ;

  private LoggedTunableNumber rollerFF;

  private LoggedTunableNumber setpointBand;
  private LoggedTunableNumber pieceIntakeCurrentThresh;

  private Debouncer pieceDebouncer = new Debouncer(0.1);
  @AutoLogOutput private boolean hasPiece = false;

  private double feedforward;

  @Getter @AutoLogOutput private double setpoint = 0.0;

  public RollerSystem(String name, RollerSystemIO io) {
    this.name = name;
    this.io = io;

    rollerkP = new LoggedTunableNumber("Roller/" + name + "/kP");
    rollerkI = new LoggedTunableNumber("Roller/" + name + "/kI");
    rollerkD = new LoggedTunableNumber("Roller/" + name + "/kD");
    rollerkS = new LoggedTunableNumber("Roller/" + name + "/kS");
    rollerkV = new LoggedTunableNumber("Roller/" + name + "/kV");
    rollerkA = new LoggedTunableNumber("Roller/" + name + "/kA");

    rollerMMV = new LoggedTunableNumber("Roller/" + name + "/MMV");
    rollerMMA = new LoggedTunableNumber("Roller/" + name + "/MMA");
    rollerMMJ = new LoggedTunableNumber("Roller/" + name + "/MMJ");

    rollerFF = new LoggedTunableNumber("Roller/" + name + "/FF", 0);
    feedforward = 0;

    setpointBand = new LoggedTunableNumber("Roller/" + name + "/setpointBand");

    pieceIntakeCurrentThresh =
        new LoggedTunableNumber("Roller/" + name + "/PieceIntakeCurrentThreshold");

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    stateTimer.start();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnected.set(!inputs.connected);

    // Update tunable numbers
    if (Constants.tuningMode) {
      if (rollerkP.hasChanged(hashCode())
          || rollerkI.hasChanged(hashCode())
          || rollerkD.hasChanged(hashCode())
          || rollerkS.hasChanged(hashCode())
          || rollerkV.hasChanged(hashCode())
          || rollerkA.hasChanged(hashCode())) {
        Slot0Configs newconfig = new Slot0Configs();
        newconfig.kP = rollerkP.get();
        newconfig.kI = rollerkI.get();
        newconfig.kD = rollerkD.get();
        newconfig.kS = rollerkS.get();
        newconfig.kV = rollerkV.get();
        newconfig.kA = rollerkA.get();
        setPID(newconfig);
      }
      if (rollerMMV.hasChanged(hashCode())
          || rollerMMA.hasChanged(hashCode())
          || rollerMMJ.hasChanged(hashCode())) {
        MotionMagicConfigs newMMconfig = new MotionMagicConfigs();
        newMMconfig.MotionMagicCruiseVelocity = rollerMMV.get();
        newMMconfig.MotionMagicAcceleration = rollerMMA.get();
        newMMconfig.MotionMagicJerk = rollerMMJ.get();
        io.setMotionMagic(newMMconfig);
      }
      if (rollerFF.hasChanged(hashCode())) {
        feedforward = rollerFF.get();
        System.out.println("\tFF changed to: " + feedforward);
      }
    }

    // Check something causing higher current
    hasPiece =
        pieceDebouncer.calculate(
            Math.abs(inputs.torqueCurrentAmps) >= pieceIntakeCurrentThresh.get());
  }

  public boolean hasPiece() {
    return hasPiece;
  }

  // must call this once and only once in robotcontainer after each RollerSystem is created
  public void setPID(Slot0Configs newconfig) {
    setPID(newconfig, null);
  }

  // must call this once and only once in robotcontainer after each RollerSystem is created
  public void setPID(Slot0Configs config0, Slot1Configs config1) {
    // slot0
    rollerkP.initDefault(config0.kP);
    rollerkI.initDefault(config0.kI);
    rollerkD.initDefault(config0.kD);
    rollerkS.initDefault(config0.kS);
    rollerkV.initDefault(config0.kV);
    rollerkA.initDefault(config0.kA);
    // slot1
    if (config1 != null) {
      rollerkP.initDefault(config1.kP);
      rollerkI.initDefault(config1.kI);
      rollerkD.initDefault(config1.kD);
      rollerkS.initDefault(config1.kS);
      rollerkV.initDefault(config1.kV);
      rollerkA.initDefault(config1.kA);
    }
    io.setPID(config0, config1);
  }

  // must call this once and only once in robotcontainer after each RollerSystem is created
  public void setMotionMagic(MotionMagicConfigs newconfig) {
    rollerMMV.initDefault(newconfig.MotionMagicCruiseVelocity);
    rollerMMA.initDefault(newconfig.MotionMagicAcceleration);
    rollerMMJ.initDefault(newconfig.MotionMagicJerk);
    io.setMotionMagic(newconfig);
  }

  public void setPieceCurrentThreshold(double threshold) {
    pieceIntakeCurrentThresh.initDefault(threshold);
  }

  public void setAtSetpointBand(double band) {
    setpointBand.initDefault(band);
  }

  @AutoLogOutput
  public void setVolts(double inputVolts) {
    setpoint = inputVolts;
    io.setVolts(inputVolts);
    Logger.recordOutput("RollerData/" + name + "/setpointVolts", setpoint);
  }

  @AutoLogOutput
  public void setSpeed(double speed) {
    // in rotations per second
    setpoint = speed;
    io.setSpeed(speed);
    Logger.recordOutput("RollerData/" + name + "/setpointSpeed", setpoint);
  }

  @AutoLogOutput
  public Command setSpeedCmd(double speed) {
    // in rotations per second
    return this.runOnce(() -> setSpeed(speed));
  }

  public void setPosition(double position) {
    setPosition(position, 0);
  }

  @AutoLogOutput
  public void setPosition(double position, int slot) {
    // in rotations
    setpoint = position;
    io.setPosition(position, feedforward, slot);
    Logger.recordOutput("RollerData/" + name + "/setpointPosition", setpoint);
    Logger.recordOutput("RollerData/" + name + "/feedforward", feedforward);
    Logger.recordOutput("RollerData/" + name + "/slot", slot);
  }

  public double getSpeed() {
    return inputs.velocityRotsPerSec;
  }

  public double getPosition() {
    return inputs.positionRotations;
  }

  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }

  public boolean atSpeed() {
    return Math.abs(setpoint - getSpeed()) < setpointBand.getAsDouble();
  }

  public boolean atPosition() {
    return Math.abs(setpoint - getPosition()) < setpointBand.getAsDouble();
  }

  public void stop() {
    io.stop();
  }

  public void zero() {
    io.zero();
  }
}
