// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RollerSystem extends SubsystemBase {
  private final String name;
  private final RollerSystemIO io;
  protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();
  private final Alert disconnected;
  protected final Timer stateTimer = new Timer();

  public RollerSystem(String name, RollerSystemIO io) {
    this.name = name;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    stateTimer.start();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnected.set(!inputs.connected);
  }

  @AutoLogOutput
  public void setVolts(double inputVolts) {
    // return startEnd(() -> io.runVolts(inputVolts), () -> io.stop());
    io.setVolts(inputVolts);
    // Logger.recordOutput(name + "/setpointVolts", inputVolts);
  }

  @AutoLogOutput
  public void setSpeed(double speed) {
    // in rotations per second
    io.setSpeed(speed);
    // Logger.recordOutput(name + "/setpointSpeed", speed);
  }

  @AutoLogOutput
  public void setPosition(double position) {
    // in rotations
    io.setPosition(position);
    // Logger.recordOutput(name + "/setpointPosition", position);
  }

  public double getSpeed() {
    return Units.radiansToRotations(inputs.velocityRadsPerSec);
  }

  public double getPosition() {
    return Units.radiansToRotations(inputs.positionRads);
  }
}
