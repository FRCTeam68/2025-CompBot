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

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.ManipulatorCommands.LOCATIONS;
import frc.robot.subsystems.ElevatorWristSubSystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import frc.robot.subsystems.RangeSensorSubSystem;
import frc.robot.subsystems.rollers.RollerSystem;

public class ManipulatorCommands {

  public enum LOCATIONS {
    INTAKE,
    L1,
    L2,
    L3,
    L4,
    L4PRIME,
    A1,
    A2,
    P1,
    PRENET,
    SHOOTNET,
    TRANSITIONING
  }

  public static LOCATIONS location;
  public static double shootSpeed;

  private ManipulatorCommands(ElevatorWristSubSystem myElevatorWrist, RollerSystem intake, RangeSensorSubSystem intake_sensor) {

    location = LOCATIONS.INTAKE;
    shootSpeed = Constants.INTAKE_SHOOTER.CORAL_INTAKE_SPEED;
  }

  public static Command intakeCmd(RollerSystem intake, RangeSensorSubSystem intake_sensor) {
    return Commands.sequence(
        new ConditionalCommand(
          intakeCoralCmd(intake, intake_sensor),
          Commands.none(),
          () -> {
            return location == LOCATIONS.INTAKE;
          }),

        new ConditionalCommand(
          intakeAlgaeCmd(intake),
          Commands.none(),
          () -> {
            return location == LOCATIONS.A1 ||
                  location == LOCATIONS.A2  ||
                  location == LOCATIONS.P1;
          }));
  }

  private static Command intakeCoralCmd(RollerSystem intake, RangeSensorSubSystem intake_sensor) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.blue, 4)),
        intake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_SPEED),
        Commands.waitUntil(() -> intake_sensor.havePiece()),
        Commands.waitSeconds(.1),
        intake.setSpeedCmd(0),
        Commands.runOnce(() -> location =LOCATIONS.INTAKE),
        Commands.runOnce(() -> shootSpeed = 0),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.blue)));
  }

  private static Command intakeAlgaeCmd(RollerSystem intake) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.blue, 4)),
        intake.setSpeedCmd(Constants.INTAKE_SHOOTER.ALGAE_INTAKE_SPEED),
        Commands.waitUntil(() -> intake.hasPiece()),
        Commands.runOnce(() -> intake.stop()),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.blue)));
  }

  public static Command shootCmd(RollerSystem intakeshooter, RangeSensorSubSystem intake_sensor, ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.red)),
        new ConditionalCommand(
          ShootAlgaeToNetCmd(myElevatorWrist, intakeshooter),
          Commands.none(),
          () -> {
            return location == LOCATIONS.PRENET;
          }),

        // shoot according to saved shooter speed
        intakeshooter.setSpeedCmd(shootSpeed),
        Commands.waitUntil(() -> intake_sensor.havePiece() == false),
        Commands.waitSeconds(1),
        intakeshooter.setSpeedCmd(0),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)),

        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        new ConditionalCommand(
            myElevatorWrist.setPositionWristCmd(Constants.WRIST.CRADLE, true),
            Commands.none(),
            () -> {
              // if location is any of these, then need to move wrist to intermediate position first
              return location == LOCATIONS.P1 ||
                    location == LOCATIONS.A1 ||
                    location == LOCATIONS.A2 ||
                    location == LOCATIONS.L4PRIME;
            }),
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.INTAKE, true),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.INTAKE, true),

        intakeCoralCmd(intakeshooter, intake_sensor)
        );
  }

  private static Command ShootAlgaeToNetCmd(ElevatorWristSubSystem myElevatorWrist, RollerSystem myShooter) {
    return Commands.sequence(
        // Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.red, 4)),
        myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.SHOOTNET, false),
        Commands.waitSeconds(.5),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.SHOOTNET, false),
        Commands.waitSeconds(.1),
        myShooter.setSpeedCmd(60),
        Commands.waitSeconds(2),
        myShooter.setSpeedCmd(0)
        // Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange))
        );
  }

  public static Command IntakeToL4primeCmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.SHOOTNET, true),
        myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.L4PRIME, true),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.L4PRIME, true),
        Commands.runOnce(() -> SmartDashboard.putString("atPosition", "L4PRIME")),
        Commands.runOnce(() -> location =LOCATIONS.L4PRIME),
        Commands.runOnce(() -> shootSpeed = Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command IntakeToL4Cmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.L4, true),
        myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.L4, true),
        Commands.runOnce(() -> SmartDashboard.putString("atPosition", "L4")),
        Commands.runOnce(() -> location =LOCATIONS.L4),
        Commands.runOnce(() -> shootSpeed = Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command IntakeToL3Cmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
      Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
      myElevatorWrist.setPositionWristCmd(Constants.WRIST.L3, true),
      myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.L3, true),
      Commands.runOnce(() -> SmartDashboard.putString("atPosition", "L3")),
      Commands.runOnce(() -> location =LOCATIONS.L3),
      Commands.runOnce(() -> shootSpeed = Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
      Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command IntakeToL2Cmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
      Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
      myElevatorWrist.setPositionWristCmd(Constants.WRIST.L2, true),
      myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.L2, true),
      Commands.runOnce(() -> SmartDashboard.putString("atPosition", "L2")),
      Commands.runOnce(() -> location =LOCATIONS.L2),
      Commands.runOnce(() -> shootSpeed = Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
      Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command IntakeToL1Cmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
      Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
      myElevatorWrist.setPositionWristCmd(Constants.WRIST.L1, true),
      myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.L1, true),
      Commands.runOnce(() -> SmartDashboard.putString("atPosition", "L1")),
      Commands.runOnce(() -> location =LOCATIONS.L1),
      Commands.runOnce(() -> shootSpeed = Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
      Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command IntakeToA2(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
      Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
      myElevatorWrist.setPositionWristCmd(Constants.WRIST.CRADLE, true),
      myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.A2, true),
      myElevatorWrist.setPositionWristCmd(Constants.WRIST.A2, true),
      Commands.runOnce(() -> SmartDashboard.putString("atPosition", "A2")),
      Commands.runOnce(() -> location =LOCATIONS.A2),
      Commands.runOnce(() -> shootSpeed = Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
      Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command IntakeToA1(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
      Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
      myElevatorWrist.setPositionWristCmd(Constants.WRIST.CRADLE, true),
      myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.A1, true),
      myElevatorWrist.setPositionWristCmd(Constants.WRIST.A1, true),
      Commands.runOnce(() -> SmartDashboard.putString("atPosition", "A1")),
      Commands.runOnce(() -> location =LOCATIONS.A1),
      Commands.runOnce(() -> shootSpeed = Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
      Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  private static Command GoToP1Cmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
      Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
      new ConditionalCommand(
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.CRADLE, true),
          Commands.none(),
          () -> {
            return location == LOCATIONS.INTAKE;
          }),
      
      myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.P1, true),
      myElevatorWrist.setPositionWristCmd(Constants.WRIST.P1, true),
      Commands.runOnce(() -> SmartDashboard.putString("atPosition", "P1")),
      Commands.runOnce(() -> location =LOCATIONS.P1),
      Commands.runOnce(() -> shootSpeed = Constants.INTAKE_SHOOTER.ALGAE_SHOOT_SPEED),
      Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command GoToPreNetCmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
      Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
      myElevatorWrist.setPositionWristCmd(Constants.WRIST.PRENET, true),
      myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.PRENET, true),
      Commands.runOnce(() -> SmartDashboard.putString("atPosition", "PRENET")),
      Commands.runOnce(() -> location =LOCATIONS.PRENET),
      Commands.runOnce(() -> shootSpeed = Constants.INTAKE_SHOOTER.ALGAE_SHOOT_SPEED),
      Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command DeployClimberCmd(RollerSystem myClimber) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setFadeAnimation(LightsSubsystem.red, 4)),
        Commands.runOnce(() -> SmartDashboard.putString("CLIMB", "deploying")),
        Commands.runOnce(() -> myClimber.setPosition(Constants.CLIMBER.GOAL), myClimber),
        Commands.waitUntil(() -> myClimber.atPosition()),
        Commands.runOnce(() -> SmartDashboard.putString("CLIMB", "DEPLOYED")),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.red)));
  }

  public static Command RetractClimberCmd(RollerSystem myClimber) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setFadeAnimation(LightsSubsystem.white, 4)),
        Commands.runOnce(() -> SmartDashboard.putString("CLIMB", "climbing")),
        Commands.runOnce(() -> myClimber.setPosition(0), myClimber),
        Commands.waitUntil(() -> myClimber.atPosition()),
        Commands.runOnce(() -> SmartDashboard.putString("CLIMB", "CLIMBED")),
        Commands.runOnce(() -> LEDSegment.all.setRainbowAnimation(4)));
  }
}
