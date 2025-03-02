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
import frc.robot.subsystems.ElevatorWristSubSystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import frc.robot.subsystems.RangeSensorSubSystem;
import frc.robot.subsystems.rollers.RollerSystem;
import org.littletonrobotics.junction.Logger;

public class ManipulatorCommands {

  private ManipulatorCommands() {}

  public static Command intakeCoralCmd(RollerSystem intake, RangeSensorSubSystem intake_sensor) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.blue, 4)),
        Commands.runOnce(
            () -> Logger.recordOutput("Manipulator/IntakeShooter/State", "IntakeCoral")),
        intake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_SPEED),
        Commands.waitUntil(() -> intake_sensor.havePiece()),
        Commands.waitSeconds(.1),
        intake.setSpeedCmd(0),
        Commands.waitSeconds(.1),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.blue)));
  }

  public static Command indexCoralCmd(RollerSystem intake, RangeSensorSubSystem intake_sensor) {
    return Commands.sequence(
        // index to have it flush to front
        Commands.runOnce(
            () -> Logger.recordOutput("Manipulator/IntakeShooter/State", "IndexCoral")),
        intake.setSpeedCmd(Constants.INTAKE_SHOOTER.COREL_INTAKE_INDEX_SPEED),
        Commands.waitUntil(() -> intake_sensor.havePiece() == false),
        intake.setSpeedCmd(0),
        Commands.waitSeconds(.1),
        Commands.runOnce(
            () ->
                intake.setPosition(
                    intake.getPosition() - Constants.INTAKE_SHOOTER.COREL_INTAKE_INDEX_REVERSE)));
  }

  public static Command intakeIndexCoralCmd(
      RollerSystem intake, RangeSensorSubSystem intake_sensor) {
    return Commands.sequence(
        intakeCoralCmd(intake, intake_sensor), indexCoralCmd(intake, intake_sensor));
  }

  public static Command shootCoralCmd(RollerSystem shooter, RangeSensorSubSystem intake_sensor) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.red)),
        Commands.runOnce(
            () -> Logger.recordOutput("Manipulator/IntakeShooter/State", "ShootCoral")),
        shooter.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
        // Commands.waitUntil(() -> intake_sensor.havePiece() == false),
        Commands.waitSeconds(0.2),
        // shooter.setSpeedCmd(0),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command intakeAlgaeA1A2Cmd(RollerSystem intake) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.blue, 4)),
        Commands.runOnce(
            () -> Logger.recordOutput("Manipulator/IntakeShooter/State", "IntakeAlgae")),
        intake.setSpeedCmd(Constants.INTAKE_SHOOTER.ALGAE_INTAKE_SPEED),
        Commands.waitUntil(() -> intake.hasPiece()),
        // change control of intake from velocity to position to hold the algae (instead of using
        // stop)
        // Commands.runOnce(() -> intake.setPosition(intake.getPosition())),
        Commands.runOnce(() -> intake.stop()),
        intake.setSpeedCmd(Constants.INTAKE_SHOOTER.ALGAE_HOLD_SPEED),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.blue)));
  }

  // public static Command shootAlgaeCmd( RollerSystem elevator, RollerSystem shooter) {
  //   return new ConditionalCommand(
  //         ShootAlgaeToNetCmd(shooter),
  //         shootAlgaeP1Cmd(shooter),
  //         () -> {
  //           // elevator greater than 11.5, we are shooting at net, not processor
  //           return elevator.getPosition() > Constants.ELEVATOR.MIN_POSITION_BLOCK4;
  //         })
  // }

  public static Command shootAlgaeP1Cmd(RollerSystem shooter) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.red)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/IntakeShooter/State", "ShootAtP1")),
        shooter.setSpeedCmd(Constants.INTAKE_SHOOTER.ALGAE_SHOOT_SPEED),
        Commands.waitSeconds(.5),
        shooter.setSpeedCmd(0),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command CoralL4Cmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "L4")),
        myElevatorWrist.setPositionCmd(Constants.ELEVATOR.L4, Constants.WRIST.L4),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command CoralL3Cmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "L3")),
        myElevatorWrist.setPositionCmd(Constants.ELEVATOR.L3, Constants.WRIST.L3),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command CoralL2Cmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "L2")),
        myElevatorWrist.setPositionCmd(Constants.ELEVATOR.L2, Constants.WRIST.L2),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command CoralL1Cmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "L1")),
        myElevatorWrist.setPositionCmd(Constants.ELEVATOR.L1, Constants.WRIST.L1),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command AlgaeToPreNetCmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "PRENET")),
        myElevatorWrist.setPositionCmd(Constants.ELEVATOR.PRENET, Constants.WRIST.PRENET),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command ShootAlgaeToNetCmd(
      ElevatorWristSubSystem myElevatorWrist, RollerSystem myShooter) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.red, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "ShootNet")),
        myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.SHOOTNET, false),
        Commands.waitSeconds(.5),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.SHOOTNET, false),
        Commands.waitSeconds(.1),
        myShooter.setSpeedCmd(60),
        Commands.waitSeconds(2),
        myShooter.setSpeedCmd(0),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command AlgaeToP1(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "P1")),
        myElevatorWrist.setPositionCmd(Constants.ELEVATOR.P1, Constants.WRIST.P1),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command AlgaeAtA2(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "A2")),
        myElevatorWrist.setPositionCmd(Constants.ELEVATOR.A2, Constants.WRIST.A2),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command AlgaeAtA1(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "A1")),
        myElevatorWrist.setPositionCmd(Constants.ELEVATOR.A1, Constants.WRIST.A1),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command DeployClimberCmd(RollerSystem myClimber) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setFadeAnimation(LightsSubsystem.red, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "deploy")),
        Commands.runOnce(() -> myClimber.setPosition(Constants.CLIMBER.GOAL), myClimber),
        Commands.waitUntil(() -> myClimber.atPosition()),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "deployed")),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.red)));
  }

  public static Command RetractClimberCmd(RollerSystem myClimber) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setFadeAnimation(LightsSubsystem.white, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "climbing")),
        Commands.runOnce(() -> myClimber.setPosition(0), myClimber),
        Commands.waitUntil(() -> myClimber.atPosition()),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "CLIMBED")),
        Commands.runOnce(() -> LEDSegment.all.setRainbowAnimation(4)));
  }
}
