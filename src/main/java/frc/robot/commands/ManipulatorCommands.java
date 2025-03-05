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
        Commands.waitSeconds(0.3),
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
        Commands.waitSeconds(.4),
        // shooter.setSpeedCmd(0),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command CoralL4Cmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "L4")),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.L4, Constants.WRIST.L4),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command CoralL3Cmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "L3")),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.L3, Constants.WRIST.L3),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command CoralL2Cmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "L2")),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.L2, Constants.WRIST.L2),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command CoralL1Cmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "L1")),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.L1, Constants.WRIST.L1),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command AlgaeToNetCmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "PRENET")),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.SHOOTNET, Constants.WRIST.PRENET),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.SHOOTNET, Constants.WRIST.SHOOTNET),
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
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.P1, Constants.WRIST.P1),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command AlgaeAtA2(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "A2")),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.A2, Constants.WRIST.A2),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command AlgaeAtA1(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "A1")),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.A1, Constants.WRIST.A1),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public static Command AlgaeCradle(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "AlgaeCradle")),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.MIN_POSITION, Constants.WRIST.SAFE),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.MIN_POSITION, Constants.WRIST.CRADLE),
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

  public static Command ZeroClimberCmd(RollerSystem myClimber) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setFadeAnimation(LightsSubsystem.red, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "ZEROING CLIMBER")),
        Commands.runOnce(() -> myClimber.setSpeedCmd(5)),
        //Commands.waitUntil(() -> myClimber.getSupplyVoltage() > Constants.CLIMBER.ZEROING_CURRENT_LIMIT), // FIXME how to monitor supply current
        Commands.runOnce(() -> myClimber.setSpeedCmd(0)),
        Commands.runOnce(() -> myClimber.zero()));
  }

  public static Command TestMoveToIntake(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "INTAKE")),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.INTAKE, Constants.WRIST.INTAKE));
  }

  public static Command TestMoveToElevatorWristZero(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "ZERO")),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.MIN_POSITION, Constants.WRIST.MIN_POSITION));
  }

  public static Command TestElevatorWristSequencing(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        // home intake
        TestMoveToIntake(myElevatorWrist),
        CoralL2Cmd(myElevatorWrist),
        TestMoveToIntake(myElevatorWrist),
        CoralL3Cmd(myElevatorWrist),
        TestMoveToIntake(myElevatorWrist),
        CoralL4Cmd(myElevatorWrist),
        TestMoveToIntake(myElevatorWrist),
        CoralL1Cmd(myElevatorWrist),
        TestMoveToIntake(myElevatorWrist),
        AlgaeToP1(myElevatorWrist),
        TestMoveToIntake(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist),
        TestMoveToIntake(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist),
        TestMoveToIntake(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist),
        TestMoveToIntake(myElevatorWrist),
        // home L2
        Commands.waitSeconds(0.3),
        CoralL2Cmd(myElevatorWrist),
        CoralL3Cmd(myElevatorWrist),
        CoralL2Cmd(myElevatorWrist),
        CoralL4Cmd(myElevatorWrist),
        CoralL2Cmd(myElevatorWrist),
        CoralL1Cmd(myElevatorWrist),
        CoralL2Cmd(myElevatorWrist),
        AlgaeToP1(myElevatorWrist),
        CoralL2Cmd(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist),
        CoralL2Cmd(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist),
        CoralL2Cmd(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist),
        CoralL2Cmd(myElevatorWrist),
        // home L3
        Commands.waitSeconds(0.3),
        CoralL3Cmd(myElevatorWrist),
        CoralL4Cmd(myElevatorWrist),
        CoralL3Cmd(myElevatorWrist),
        CoralL1Cmd(myElevatorWrist),
        CoralL3Cmd(myElevatorWrist),
        AlgaeToP1(myElevatorWrist),
        CoralL3Cmd(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist),
        CoralL3Cmd(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist),
        CoralL3Cmd(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist),
        CoralL3Cmd(myElevatorWrist),
        // home L4
        Commands.waitSeconds(0.3),
        CoralL4Cmd(myElevatorWrist),
        CoralL1Cmd(myElevatorWrist),
        CoralL4Cmd(myElevatorWrist),
        AlgaeToP1(myElevatorWrist),
        CoralL4Cmd(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist),
        CoralL4Cmd(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist),
        CoralL4Cmd(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist),
        CoralL4Cmd(myElevatorWrist),
        // home L1
        Commands.waitSeconds(0.3),
        CoralL1Cmd(myElevatorWrist),
        AlgaeToP1(myElevatorWrist),
        CoralL1Cmd(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist),
        CoralL1Cmd(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist),
        CoralL1Cmd(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist),
        CoralL1Cmd(myElevatorWrist),
        // home P1
        Commands.waitSeconds(0.3),
        AlgaeToP1(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist),
        AlgaeToP1(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist),
        AlgaeToP1(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist),
        AlgaeToP1(myElevatorWrist),
        // home A1
        Commands.waitSeconds(0.3),
        AlgaeAtA1(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist),
        // home A2
        Commands.waitSeconds(0.3),
        AlgaeAtA2(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist),
        // return to zero
        Commands.waitSeconds(0.3),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.MIN_POSITION, Constants.WRIST.MIN_POSITION));
  }
}
