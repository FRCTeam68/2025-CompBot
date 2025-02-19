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

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import frc.robot.subsystems.RangeSensorSystem;
import frc.robot.subsystems.rollers.RollerSystem;

public class ManipulatorCommands {

  private ManipulatorCommands() {}

  public static Command intakeCoral(RollerSystem intake, RangeSensorSystem intake_sensor) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.blue, .5)),
        intake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_SPEED),
        Commands.waitUntil(() -> intake_sensor.havePiece()),
        Commands.runOnce(() -> intake.setSpeedCmd(0)),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.blue)));
  }

  public static Command shootCoral(RollerSystem shooter, RangeSensorSystem intake_sensor) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.red)),
        shooter.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
        Commands.waitUntil(() -> intake_sensor.havePiece() == false),
        Commands.waitSeconds(2),
        Commands.runOnce(() -> shooter.setSpeedCmd(0)),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command intakeAlgaeA1A2(RollerSystem intake) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.blue, .5)),
        intake.setSpeedCmd(Constants.INTAKE_SHOOTER.ALGAE_INTAKE_SPEED),
        Commands.waitUntil(() -> intake.hasPiece()),
        // change control of intake from velocity to position to hold the algae (instead of using
        // stop)
        Commands.runOnce(() -> intake.setPosition(intake.getPosition())),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.blue)));
  }

  public static Command shootAlgaeP1(RollerSystem shooter) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.red)),
        shooter.setSpeedCmd(Constants.INTAKE_SHOOTER.ALGAE_SHOOT_SPEED),
        Commands.waitSeconds(2),
        Commands.runOnce(() -> shooter.setSpeedCmd(0)),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }
}
