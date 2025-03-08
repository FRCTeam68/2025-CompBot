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
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorWristSubSystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import frc.robot.subsystems.RangeSensorSubSystem;
import frc.robot.subsystems.rollers.RollerSystem;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class ManipulatorCommands {

  private ManipulatorCommands() {}

  public static Command intakeCmd(
      RollerSystem intake,
      RangeSensorSubSystem intake_sensor,
      ElevatorWristSubSystem myElevatorWrist) {
    return new DeferredCommand(
        () -> {
          // initialization
          Command command;
          Command ledIntaking =
              Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.blue, 4));
          Command ledHaveObject =
              Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.blue));
          command = Commands.none();
          if (myElevatorWrist.getWristAngle() <= Constants.WRIST.MAX_SLOT1_TO_ELEVATE) {
            ///// INTAKE CORAL /////
            command =
                Commands.sequence(
                    CoralIntakePositionCmd(myElevatorWrist),
                    Commands.runOnce(
                        () ->
                            Logger.recordOutput("Manipulator/IntakeShooter/State", "IntakeCoral")),
                    intake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_SPEED),
                    Commands.waitUntil(() -> intake_sensor.havePiece()),
                    ledHaveObject,
                    Commands.waitSeconds(.05),
                    intake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_INDEX_SPEED),
                    Commands.waitUntil(() -> intake_sensor.havePiece() == false),
                    intake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_INDEX_SPEED * -1),
                    Commands.waitUntil(() -> intake_sensor.havePiece()),
                    Commands.runOnce(
                        () ->
                            intake.setPosition(
                                intake.getPosition()
                                    - Constants.INTAKE_SHOOTER.CORAL_INTAKE_INDEX_REVERSE)));
          } else {
            ///// INTAKE ALGAE /////
            command =
                Commands.sequence(
                    Commands.runOnce(
                        () ->
                            Logger.recordOutput("Manipulator/IntakeShooter/State", "IntakeAlgae")),
                    intake.setSpeedCmd(Constants.INTAKE_SHOOTER.ALGAE_INTAKE_SPEED),
                    Commands.waitUntil(() -> intake.hasPiece()),
                    ledHaveObject,
                    Commands.runOnce(() -> intake.setPosition(intake.getPosition())));
          }
          // execute sequence
          return (ledIntaking.andThen(command));
        },
        Set.of(intake, intake_sensor, myElevatorWrist));
  }

  public static Command shootCmd(RollerSystem shooter, ElevatorWristSubSystem myElevatorWrist) {
    return new DeferredCommand(
        () -> {
          // initialization
          Command command;
          Command ledShooting =
              Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.red));
          Command ledShot = Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange));
          command = Commands.none();
          if (myElevatorWrist.getWristAngle() <= Constants.WRIST.MAX_SLOT1_TO_ELEVATE) {
            ///// SHOOT CORAL /////
            command =
                Commands.sequence(
                    Commands.runOnce(
                        () -> Logger.recordOutput("Manipulator/IntakeShooter/State", "ShootCoral")),
                    shooter.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
                    Commands.waitSeconds(0.5),
                    shooter.setSpeedCmd(0),
                    ledShot);
          } else {
            ///// SHOOT ALGAE /////
            command =
                Commands.sequence(
                    Commands.runOnce(
                        () -> Logger.recordOutput("Manipulator/IntakeShooter/State", "ShootAlgae")),
                    shooter.setSpeedCmd(Constants.INTAKE_SHOOTER.ALGAE_SHOOT_SPEED),
                    Commands.waitSeconds(.5),
                    // shooter.setSpeedCmd(0),
                    Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
          }
          // execute sequence
          return (ledShooting.andThen(command));
        },
        Set.of(shooter, myElevatorWrist));
  }
  /*
    public static Command intakeCoralCmd(RollerSystem intake, RangeSensorSubSystem intake_sensor, ElevatorWristSubSystem myElevatorWrist) {
      return Commands.sequence(
        CoralIntakePositionCmd(myElevatorWrist),
          Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.blue, 4)),
          Commands.runOnce(
              () -> Logger.recordOutput("Manipulator/IntakeShooter/State", "IntakeCoral")),
          intake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_SPEED),
          Commands.waitUntil(() -> intake_sensor.havePiece()),
          Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.blue)),
          Commands.waitSeconds(.05),
          intake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_INDEX_SPEED),
          Commands.waitUntil(() -> intake_sensor.havePiece() == false),
          intake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_INDEX_SPEED * -1),
          Commands.waitUntil(() -> intake_sensor.havePiece()),
          Commands.runOnce(() -> intake.setPosition(intake.getPosition() - Constants.INTAKE_SHOOTER.CORAL_INTAKE_INDEX_REVERSE))
          );
    }

    public static Command indexCoralCmd(RollerSystem intake, RangeSensorSubSystem intake_sensor) {
      return Commands.sequence(
          // index to have it flush to front
          Commands.runOnce(
              () -> Logger.recordOutput("Manipulator/IntakeShooter/State", "IndexCoral")),
          intake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_INDEX_SPEED),
          Commands.waitUntil(() -> intake_sensor.havePiece() == false),
          intake.setSpeedCmd(0),
          Commands.waitSeconds(.1),
          Commands.runOnce(
              () ->
                  intake.setPosition(
                      intake.getPosition() - Constants.INTAKE_SHOOTER.CORAL_INTAKE_INDEX_REVERSE)));
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
  */
  public static Command CoralL4Cmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "L4")),
        myElevatorWrist.setPositionCmdNewIf(Constants.ELEVATOR.L4, Constants.WRIST.L4));
  }

  public static Command CoralL3Cmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "L3")),
        myElevatorWrist.setPositionCmdNewIf(Constants.ELEVATOR.L3, Constants.WRIST.L3));
  }

  public static Command CoralL2Cmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "L2")),
        myElevatorWrist.setPositionCmdNewIf(Constants.ELEVATOR.L2, Constants.WRIST.L2));
  }

  public static Command CoralL1Cmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "L1")),
        myElevatorWrist.setPositionCmdNewIf(Constants.ELEVATOR.L1, Constants.WRIST.L1));
  }

  public static Command CoralIntakePositionCmd(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "INTAKE")),
        myElevatorWrist.setPositionCmdNewIf(
            Constants.ELEVATOR.MIN_POSITION, Constants.WRIST.INTAKE));
  }

  public static Command AlgaeToNetCmd(
      ElevatorWristSubSystem myElevatorWrist, Boolean algaeCradleFlag) {
    return Commands.sequence(
        Commands.waitUntil(() -> algaeCradleFlag == false),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "NET")),
        myElevatorWrist.setPositionCmdNewIf(Constants.ELEVATOR.SHOOTNET, Constants.WRIST.PRENET),
        myElevatorWrist.setPositionCmdNewIf(Constants.ELEVATOR.SHOOTNET, Constants.WRIST.SHOOTNET));
  }
  /*
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
  */
  public static Command AlgaeToP1(ElevatorWristSubSystem myElevatorWrist, Boolean algaeCradleFlag) {
    return Commands.sequence(
        Commands.waitUntil(() -> algaeCradleFlag == false),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "P1")),
        myElevatorWrist.setPositionCmdNewIf(Constants.ELEVATOR.P1, Constants.WRIST.P1));
  }

  public static Command AlgaeAtA2(ElevatorWristSubSystem myElevatorWrist, Boolean algaeCradleFlag) {
    return Commands.sequence(
        Commands.waitUntil(() -> algaeCradleFlag == false),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "A2")),
        myElevatorWrist.setPositionCmdNewIf(Constants.ELEVATOR.A2, Constants.WRIST.A2));
  }

  public static Command AlgaeAtA1(ElevatorWristSubSystem myElevatorWrist, Boolean algaeCradleFlag) {
    return Commands.sequence(
        Commands.waitUntil(() -> algaeCradleFlag == false),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "A1")),
        myElevatorWrist.setPositionCmdNewIf(Constants.ELEVATOR.A1, Constants.WRIST.A1));
  }

  public static Command AlgaeCradle(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "AlgaeCradle")),
        myElevatorWrist.setPositionCmdNewIf(Constants.ELEVATOR.MIN_POSITION, Constants.WRIST.SAFE),
        myElevatorWrist.setPositionCmdNewIf(
            Constants.ELEVATOR.MIN_POSITION, Constants.WRIST.CRADLE));
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
        Commands.runOnce(() -> myClimber.setSpeedCmd(Constants.CLIMBER.ZEROING_SPEED)),
        Commands.waitUntil(
            () -> myClimber.getSupplyCurrent() > Constants.CLIMBER.ZEROING_CURRENT_LIMIT),
        Commands.runOnce(() -> myClimber.setSpeedCmd(0)),
        Commands.runOnce(() -> myClimber.zero()));
  }

  public static Command MoveToElevatorWristZero(ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/State", "ZERO")),
        myElevatorWrist.setPositionCmdNewIf(
            Constants.ELEVATOR.MIN_POSITION, Constants.WRIST.MIN_POSITION));
  }

  public static Command TestElevatorWristSequencing(ElevatorWristSubSystem myElevatorWrist) {
    final boolean testAlgaeCradleFlag = false;
    return Commands.sequence(
        // home intake
        CoralIntakePositionCmd(myElevatorWrist),
        CoralL2Cmd(myElevatorWrist),
        CoralIntakePositionCmd(myElevatorWrist),
        CoralL3Cmd(myElevatorWrist),
        CoralIntakePositionCmd(myElevatorWrist),
        CoralL4Cmd(myElevatorWrist),
        CoralIntakePositionCmd(myElevatorWrist),
        CoralL1Cmd(myElevatorWrist),
        CoralIntakePositionCmd(myElevatorWrist),
        AlgaeToP1(myElevatorWrist, testAlgaeCradleFlag),
        CoralIntakePositionCmd(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist, testAlgaeCradleFlag),
        CoralIntakePositionCmd(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist, testAlgaeCradleFlag),
        CoralIntakePositionCmd(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist, testAlgaeCradleFlag),
        CoralIntakePositionCmd(myElevatorWrist),
        AlgaeCradle(myElevatorWrist),
        CoralIntakePositionCmd(myElevatorWrist),
        // home L2
        Commands.waitSeconds(0.3),
        CoralL2Cmd(myElevatorWrist),
        CoralL3Cmd(myElevatorWrist),
        CoralL2Cmd(myElevatorWrist),
        CoralL4Cmd(myElevatorWrist),
        CoralL2Cmd(myElevatorWrist),
        CoralL1Cmd(myElevatorWrist),
        CoralL2Cmd(myElevatorWrist),
        AlgaeToP1(myElevatorWrist, testAlgaeCradleFlag),
        CoralL2Cmd(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist, testAlgaeCradleFlag),
        CoralL2Cmd(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist, testAlgaeCradleFlag),
        CoralL2Cmd(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist, testAlgaeCradleFlag),
        CoralL2Cmd(myElevatorWrist),
        AlgaeCradle(myElevatorWrist),
        CoralL2Cmd(myElevatorWrist),
        // home L3
        Commands.waitSeconds(0.3),
        CoralL3Cmd(myElevatorWrist),
        CoralL4Cmd(myElevatorWrist),
        CoralL3Cmd(myElevatorWrist),
        CoralL1Cmd(myElevatorWrist),
        CoralL3Cmd(myElevatorWrist),
        AlgaeToP1(myElevatorWrist, testAlgaeCradleFlag),
        CoralL3Cmd(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist, testAlgaeCradleFlag),
        CoralL3Cmd(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist, testAlgaeCradleFlag),
        CoralL3Cmd(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist, testAlgaeCradleFlag),
        CoralL3Cmd(myElevatorWrist),
        AlgaeCradle(myElevatorWrist),
        CoralL3Cmd(myElevatorWrist),
        // home L4
        Commands.waitSeconds(0.3),
        CoralL4Cmd(myElevatorWrist),
        CoralL1Cmd(myElevatorWrist),
        CoralL4Cmd(myElevatorWrist),
        AlgaeToP1(myElevatorWrist, testAlgaeCradleFlag),
        CoralL4Cmd(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist, testAlgaeCradleFlag),
        CoralL4Cmd(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist, testAlgaeCradleFlag),
        CoralL4Cmd(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist, testAlgaeCradleFlag),
        CoralL4Cmd(myElevatorWrist),
        AlgaeCradle(myElevatorWrist),
        CoralL4Cmd(myElevatorWrist),
        // home L1
        Commands.waitSeconds(0.3),
        CoralL1Cmd(myElevatorWrist),
        AlgaeToP1(myElevatorWrist, testAlgaeCradleFlag),
        CoralL1Cmd(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist, testAlgaeCradleFlag),
        CoralL1Cmd(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist, testAlgaeCradleFlag),
        CoralL1Cmd(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist, testAlgaeCradleFlag),
        CoralL1Cmd(myElevatorWrist),
        AlgaeCradle(myElevatorWrist),
        CoralL1Cmd(myElevatorWrist),
        // home P1
        Commands.waitSeconds(0.3),
        AlgaeToP1(myElevatorWrist, testAlgaeCradleFlag),
        AlgaeAtA1(myElevatorWrist, testAlgaeCradleFlag),
        AlgaeToP1(myElevatorWrist, testAlgaeCradleFlag),
        AlgaeAtA2(myElevatorWrist, testAlgaeCradleFlag),
        AlgaeToP1(myElevatorWrist, testAlgaeCradleFlag),
        AlgaeToNetCmd(myElevatorWrist, testAlgaeCradleFlag),
        AlgaeToP1(myElevatorWrist, testAlgaeCradleFlag),
        AlgaeCradle(myElevatorWrist),
        AlgaeToP1(myElevatorWrist, testAlgaeCradleFlag),
        // home A1
        Commands.waitSeconds(0.3),
        AlgaeAtA1(myElevatorWrist, testAlgaeCradleFlag),
        AlgaeAtA2(myElevatorWrist, testAlgaeCradleFlag),
        AlgaeAtA1(myElevatorWrist, testAlgaeCradleFlag),
        AlgaeToNetCmd(myElevatorWrist, testAlgaeCradleFlag),
        AlgaeAtA1(myElevatorWrist, testAlgaeCradleFlag),
        AlgaeCradle(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist, testAlgaeCradleFlag),
        // home A2
        Commands.waitSeconds(0.3),
        AlgaeAtA2(myElevatorWrist, testAlgaeCradleFlag),
        AlgaeToNetCmd(myElevatorWrist, testAlgaeCradleFlag),
        AlgaeAtA2(myElevatorWrist, testAlgaeCradleFlag),
        AlgaeCradle(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist, testAlgaeCradleFlag),
        // home net
        Commands.waitSeconds(0.3),
        AlgaeToNetCmd(myElevatorWrist, testAlgaeCradleFlag),
        AlgaeCradle(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist, testAlgaeCradleFlag),
        // return to zero
        Commands.waitSeconds(0.3),
        MoveToElevatorWristZero(myElevatorWrist));
  }
}
