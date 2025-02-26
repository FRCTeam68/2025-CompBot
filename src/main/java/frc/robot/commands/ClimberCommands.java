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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import frc.robot.subsystems.rollers.RollerSystem;

public class ClimberCommands {

  private RollerSystem myClimber;

  public ClimberCommands(RollerSystem climber) {
    myClimber = climber;
  }

  public Command DeployClimberCmd() {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setFadeAnimation(LightsSubsystem.red, 4)),
        Commands.runOnce(() -> SmartDashboard.putString("CLIMB", "deploying")),
        Commands.runOnce(() -> myClimber.setPosition(Constants.CLIMBER.GOAL), myClimber),
        Commands.waitUntil(() -> myClimber.atPosition()),
        Commands.runOnce(() -> SmartDashboard.putString("CLIMB", "DEPLOYED")),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.red)));
  }

  public Command RetractClimberCmd() {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setFadeAnimation(LightsSubsystem.white, 4)),
        Commands.runOnce(() -> SmartDashboard.putString("CLIMB", "climbing")),
        Commands.runOnce(() -> myClimber.setPosition(0), myClimber),
        Commands.waitUntil(() -> myClimber.atPosition()),
        Commands.runOnce(() -> SmartDashboard.putString("CLIMB", "CLIMBED")),
        Commands.runOnce(() -> LEDSegment.all.setRainbowAnimation(4)));
  }
}
