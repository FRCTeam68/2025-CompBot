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
import frc.robot.Constants.LEDColor;
import frc.robot.Constants.LEDSegment;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorWristSubSystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.RangeSensorSubSystem;
import frc.robot.subsystems.rollers.RollerSystem;
import java.util.Set;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ManipulatorCommands {

  @Getter @AutoLogOutput public static boolean havePiece = false;
  @Getter @AutoLogOutput public static boolean indexing = false;

  private ManipulatorCommands() {}

  public static Command intakeCmd(
      RollerSystem myIntake,
      RollerSystem myIntakeLow,
      ElevatorWristSubSystem myElevatorWrist,
      RangeSensorSubSystem intake_sensor) {
    return new DeferredCommand(
        () -> {
          // initialization
          Command command;
          Command ledIntaking =
              Commands.runOnce(
                  () -> LightsSubsystem.setBandAnimation(LEDColor.BLUE, 4, LEDSegment.ALL));
          Command ledHaveObject =
              Commands.runOnce(() -> LightsSubsystem.setColor(LEDColor.BLUE, LEDSegment.ALL));
          Command haveObjectFlag =
              Commands.parallel(
                  Commands.runOnce(() -> havePiece = true),
                  Commands.runOnce(() -> indexing = false));
          command = Commands.none();
          if (Constants.WRIST.POSITION_SCORING_ELEMENT == "Algae"
              || Constants.WRIST.POSITION_SCORING_ELEMENT == "AlgaeNet") {
            ///// INTAKE ALGAE /////
            command =
                Commands.sequence(
                    Commands.runOnce(() -> havePiece = false),
                    Commands.runOnce(
                        () -> Logger.recordOutput("Manipulator/IntakeShooterState", "IntakeAlgae")),
                    myIntake.setSpeedCmd(Constants.INTAKE_SHOOTER.ALGAE_INTAKE_SPEED),
                    myIntakeLow.setSpeedCmd(Constants.INTAKE_SHOOTER_LOW.ALGAE_INTAKE_SPEED),
                    Commands.waitUntil(() -> myIntake.hasPiece()),
                    ledHaveObject,
                    myIntake.setSpeedCmd(Constants.INTAKE_SHOOTER.ALGAE_HOLD_SPEED),
                    myIntakeLow.setSpeedCmd(Constants.INTAKE_SHOOTER_LOW.ALGAE_HOLD_SPEED));
          } else {
            ///// INTAKE CORAL /////
            if (intake_sensor.isDetected()) {
              command =
                  Commands.parallel(
                      Commands.runOnce(
                          () ->
                              Logger.recordOutput(
                                  "Manipulator/IntakeShooterState", "AlreadyHasCoral")),
                      CoralIntakePositionCmd(myIntakeLow, myElevatorWrist),
                      myIntake.setSpeedCmd(0),
                      myIntakeLow.setSpeedCmd(0),
                      Commands.runOnce(() -> indexing = true),
                      Commands.runOnce(() -> havePiece = true));
            } else {
              command =
                  Commands.sequence(
                      myIntakeLow.setSpeedCmd(0),
                      Commands.runOnce(() -> havePiece = false),
                      Commands.runOnce(() -> indexing = false),
                      Commands.runOnce(
                          () ->
                              Logger.recordOutput("Manipulator/IntakeShooterState", "IntakeCoral")),
                      CoralIntakePositionCmd(myIntakeLow, myElevatorWrist),
                      myIntake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_SPEED),
                      Commands.waitUntil(() -> intake_sensor.isDetected()),
                      Commands.runOnce(() -> indexing = true),
                      Commands.waitSeconds(.03),
                      myIntake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_INDEX_SPEED),
                      Commands.waitUntil(() -> intake_sensor.isDetected() == false),
                      myIntake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_INDEX_SPEED * -1),
                      Commands.waitUntil(() -> intake_sensor.isDetected()),
                      Commands.runOnce(
                          () ->
                              myIntake.setPosition(
                                  myIntake.getPosition()
                                      - Constants.INTAKE_SHOOTER.CORAL_INTAKE_INDEX_REVERSE)),
                      ledHaveObject);
            }
          }
          // execute sequence
          return ledIntaking.andThen(command).andThen(haveObjectFlag);
        },
        Set.of(myIntake, intake_sensor, myElevatorWrist));
  }

  public static Command shootCmd(
      RollerSystem myIntake, RollerSystem myIntakeLow, ElevatorWristSubSystem myElevatorWrist) {
    return new DeferredCommand(
        () -> {
          // initialization
          Command command;
          Command ledShooting =
              Commands.sequence(
                  // need to set false before setting LED green so elevatorWrist periodic does not
                  // turn back to red during the shoot command
                  Commands.runOnce(() -> myElevatorWrist.setLookingToShoot(false)),
                  Commands.runOnce(() -> LightsSubsystem.setColor(LEDColor.GREEN, LEDSegment.ALL)));
          Command afterShot =
              Commands.parallel(
                  Commands.runOnce(() -> havePiece = false),
                  Commands.runOnce(() -> indexing = false),
                  Commands.runOnce(() -> LightsSubsystem.disableLEDs(LEDSegment.ALL)),
                  myIntake.setSpeedCmd(0),
                  myIntakeLow.setSpeedCmd(0));
          command = Commands.none();
          if (Constants.WRIST.POSITION_SCORING_ELEMENT == "Algae") {
            ///// SHOOT ALGAE PROCESSOR /////
            command =
                Commands.sequence(
                    Commands.runOnce(
                        () ->
                            Logger.recordOutput(
                                "Manipulator/IntakeShooterState", "ShootProcessor")),
                    myIntake.setSpeedCmd(Constants.INTAKE_SHOOTER.ALGAE_SHOOT_SPEED),
                    myIntakeLow.setSpeedCmd(Constants.INTAKE_SHOOTER_LOW.ALGAE_SHOOT_SPEED),
                    Commands.waitSeconds(Constants.INTAKE_SHOOTER.ALGAE_SHOOT_TIMEOUT));
          } else if (Constants.WRIST.POSITION_SCORING_ELEMENT == "AlgaeNet") {
            ///// SHOOT ALGAE NET /////
            command =
                Commands.parallel(
                    Commands.runOnce(
                        () -> Logger.recordOutput("Manipulator/ElevatorWristState", "ShootNet")),
                    myElevatorWrist.setPositionCmdNew(
                        Constants.ELEVATOR.SHOOTNET, Constants.WRIST.SHOOTNET, 1),
                    Commands.sequence(
                        Commands.waitSeconds(Constants.INTAKE_SHOOTER.ALGAE_NET_SHOOT_DELAY),
                        myIntake.setSpeedCmd(Constants.INTAKE_SHOOTER.ALGAE_NET_SHOOT_SPEED),
                        myIntakeLow.setSpeedCmd(Constants.INTAKE_SHOOTER_LOW.ALGAE_NET_SHOOT_SPEED),
                        Commands.waitSeconds(Constants.INTAKE_SHOOTER.ALGAE_SHOOT_TIMEOUT)));
          } else if (Constants.WRIST.POSITION_SCORING_ELEMENT == "CoralL1") {
            ///// SHOOT CORAL L1 /////
            command =
                Commands.sequence(
                    Commands.runOnce(
                        () ->
                            Logger.recordOutput("Manipulator/IntakeShooterState", "ShootCoralL1")),
                    myIntake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_L1_SHOOT_SPEED),
                    Commands.waitSeconds(Constants.INTAKE_SHOOTER.CORAL_L1_SHOOT_TIMEOUT));

            ///// SHOOT CORAL L1 PIVOT /////
            // command =
            //     Commands.sequence(
            //         Commands.parALLel(
            //             Commands.runOnce(
            //                 () ->
            //                     Logger.recordOutput("Manipulator/IntakeShooterState",
            // "ShootCoralL1")),
            //             myIntake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
            //             myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.L1_FINAL,
            // Constants.WRIST.L1)),
            //         Commands.waitSeconds(Constants.INTAKE_SHOOTER.CORAL_L1_SHOOT_TIMEOUT),
            //         myIntake.setSpeedCmd(0),
            //         myIntakeLow.setSpeedCmd(0));
          } else {
            ///// SHOOT CORAL L2 - L4 /////
            command =
                Commands.sequence(
                    Commands.runOnce(
                        () -> Logger.recordOutput("Manipulator/IntakeShooterState", "ShootCoral")),
                    myIntake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
                    Commands.waitSeconds(Constants.INTAKE_SHOOTER.CORAL_SHOOT_TIMEOUT));
          }
          // execute sequence
          return ledShooting.andThen(command).andThen(afterShot);
        },
        Set.of(myIntake, myElevatorWrist));
  }

  public static Command CoralL4Cmd(
      RollerSystem myIntakeLow, ElevatorWristSubSystem myElevatorWrist) {
    return Commands.parallel(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "Coral"),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "L4")),
        Commands.either(
            Commands.sequence(
                myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.L4, Constants.WRIST.L4),
                Commands.runOnce(() -> myElevatorWrist.setLookingToShoot(true))),
            Commands.none(),
            () -> {
              return ManipulatorCommands.havePiece || RobotContainer.m_climberBump;
            }));
  }

  public static Command CoralL3Cmd(
      RollerSystem myIntakeLow, ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "Coral"),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "L3")),
        Commands.either(
            Commands.sequence(
                myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.L3, Constants.WRIST.L3),
                Commands.runOnce(() -> myElevatorWrist.setLookingToShoot(true))),
            Commands.none(),
            () -> {
              return ManipulatorCommands.havePiece || RobotContainer.m_climberBump;
            }));
  }

  public static Command CoralL2Cmd(
      RollerSystem myIntakeLow, ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "Coral"),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "L2")),
        Commands.either(
            Commands.sequence(
                myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.L2, Constants.WRIST.L2),
                Commands.runOnce(() -> myElevatorWrist.setLookingToShoot(true))),
            Commands.none(),
            () -> {
              return ManipulatorCommands.havePiece || RobotContainer.m_climberBump;
            }));
  }

  public static Command CoralL1Cmd(
      RollerSystem myIntakeLow, ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "CoralL1"),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "L1")),
        Commands.either(
            myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.L1, Constants.WRIST.L1, 1),
            Commands.none(),
            () -> {
              return ManipulatorCommands.havePiece || RobotContainer.m_climberBump;
            }));
  }

  public static Command CoralIntakePositionCmd(
      RollerSystem myIntakeLow, ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "Coral"),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "INTAKE")),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.INTAKE, Constants.WRIST.INTAKE));
  }

  public static Command AlgaeToNetCmd(
      RollerSystem myIntakeLow, ElevatorWristSubSystem myElevatorWrist, Boolean algaeCradleFlag) {
    return Commands.sequence(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "AlgaeNet"),
        Commands.waitUntil(() -> algaeCradleFlag == false),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "NET")),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.PRENET, Constants.WRIST.PRENET, 1));
  }

  public static Command AlgaeToP1(
      RollerSystem myIntakeLow, ElevatorWristSubSystem myElevatorWrist, Boolean algaeCradleFlag) {
    return Commands.sequence(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "Algae"),
        Commands.waitUntil(() -> algaeCradleFlag == false),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "P1")),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.P1, Constants.WRIST.P1));
  }

  public static Command AlgaeAtA2(
      RollerSystem myIntakeLow, ElevatorWristSubSystem myElevatorWrist, Boolean algaeCradleFlag) {
    return Commands.sequence(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "Algae"),
        Commands.waitUntil(() -> algaeCradleFlag == false),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "A2")),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.A2, Constants.WRIST.A2));
  }

  public static Command AlgaeAtA1(
      RollerSystem myIntakeLow, ElevatorWristSubSystem myElevatorWrist, Boolean algaeCradleFlag) {
    return Commands.sequence(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "Algae"),
        Commands.waitUntil(() -> algaeCradleFlag == false),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "A1")),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.A1, Constants.WRIST.A1));
  }

  public static Command AlgaeCradle(
      RollerSystem myIntakeLow, ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "Algae"),
        Commands.runOnce(
            () -> Logger.recordOutput("Manipulator/ElevatorWristState", "AlgaeCradle")),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.MIN_POSITION, Constants.WRIST.SAFE),
        myElevatorWrist.setPositionCmdNew(Constants.ELEVATOR.MIN_POSITION, Constants.WRIST.CRADLE));
  }

  public static Command DeployClimberCmd(RollerSystem myClimber) {
    return Commands.sequence(
        Commands.runOnce(() -> LightsSubsystem.setFadeAnimation(LEDColor.RED, 4, LEDSegment.ALL)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "deploying")),
        Commands.runOnce(() -> myClimber.setPosition(Constants.CLIMBER.DEPLOY), myClimber),
        Commands.waitUntil(() -> myClimber.atPosition()),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "deployed")),
        Commands.runOnce(() -> LightsSubsystem.setColor(LEDColor.RED, LEDSegment.ALL)));
  }

  public static Command RetractClimberCmd(RollerSystem myClimber) {
    return Commands.sequence(
        Commands.runOnce(() -> LightsSubsystem.setFadeAnimation(LEDColor.GREEN, 4, LEDSegment.ALL)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "climbing")),
        Commands.runOnce(() -> myClimber.setPosition(Constants.CLIMBER.RETRACT), myClimber),
        Commands.waitUntil(() -> myClimber.atPosition()),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "CLIMBED")),
        Commands.runOnce(() -> LightsSubsystem.setRainbowAnimation(4, LEDSegment.ALL)));
  }

  public static Command climberToZeroCmd(RollerSystem myClimber) {
    return Commands.sequence(
        Commands.runOnce(() -> LightsSubsystem.setFadeAnimation(LEDColor.RED, 4, LEDSegment.ALL)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "to zero")),
        Commands.runOnce(() -> myClimber.setPosition(0), myClimber),
        Commands.waitUntil(() -> myClimber.atPosition()),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "at zero")),
        Commands.runOnce(() -> LightsSubsystem.setRainbowAnimation(4, LEDSegment.ALL)));
  }

  // NO CHECKS
  public static Command BumpClimberCmd(double bump, RollerSystem myClimber) {
    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "bumping")),
        Commands.runOnce(() -> myClimber.setPosition(myClimber.getPosition() + bump), myClimber),
        Commands.waitUntil(() -> myClimber.atPosition()),
        Commands.runOnce(() -> myClimber.zero()));
  }

  public static Command ElevatorWristZeroCmd(
      RollerSystem myIntakeLow, ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "Null"),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "ZERO")),
        myElevatorWrist.setPositionCmdNew(
            Constants.ELEVATOR.MIN_POSITION, Constants.WRIST.MIN_POSITION));
  }

  public static Command FunctionalTest(
      RollerSystem myIntake,
      RollerSystem myIntakeLow,
      ElevatorWristSubSystem myElevatorWrist,
      RangeSensorSubSystem intake_sensor,
      RollerSystem myClimber) {
    return Commands.sequence(
        CoralIntakePositionCmd(myIntakeLow, myElevatorWrist),
        intakeCmd(myIntake, myIntakeLow, myElevatorWrist, intake_sensor),
        CoralL4Cmd(myIntakeLow, myElevatorWrist),
        Commands.waitSeconds(0.5),
        CoralL1Cmd(myIntakeLow, myElevatorWrist),
        Commands.waitSeconds(0.5),
        shootCmd(myIntake, myIntakeLow, myElevatorWrist),
        Commands.waitSeconds(1),
        AlgaeToP1(myIntakeLow, myElevatorWrist, false),
        intakeCmd(myIntake, myIntakeLow, myElevatorWrist, intake_sensor),
        AlgaeToNetCmd(myIntakeLow, myElevatorWrist, false),
        Commands.waitSeconds(0.5),
        AlgaeToP1(myIntakeLow, myElevatorWrist, false),
        Commands.waitSeconds(0.5),
        shootCmd(myIntake, myIntakeLow, myElevatorWrist),
        Commands.waitSeconds(1),
        ElevatorWristZeroCmd(myIntakeLow, myElevatorWrist),
        Commands.waitSeconds(0.5),
        DeployClimberCmd(myClimber),
        Commands.waitSeconds(0.5),
        // RetractClimberCmd(myClimber),   // this will latch so we would not want to go to zero
        climberToZeroCmd(myClimber));
  }

  public static Command TestElevatorWristSequencing(
      RollerSystem myIntakeLow, ElevatorWristSubSystem myElevatorWrist) {
    return Commands.sequence(
        // home intake
        CoralIntakePositionCmd(myIntakeLow, myElevatorWrist),
        CoralL2Cmd(myIntakeLow, myElevatorWrist),
        CoralIntakePositionCmd(myIntakeLow, myElevatorWrist),
        CoralL3Cmd(myIntakeLow, myElevatorWrist),
        CoralIntakePositionCmd(myIntakeLow, myElevatorWrist),
        CoralL4Cmd(myIntakeLow, myElevatorWrist),
        CoralIntakePositionCmd(myIntakeLow, myElevatorWrist),
        CoralL1Cmd(myIntakeLow, myElevatorWrist),
        CoralIntakePositionCmd(myIntakeLow, myElevatorWrist),
        AlgaeToP1(myIntakeLow, myElevatorWrist, false),
        CoralIntakePositionCmd(myIntakeLow, myElevatorWrist),
        AlgaeAtA1(myIntakeLow, myElevatorWrist, false),
        CoralIntakePositionCmd(myIntakeLow, myElevatorWrist),
        AlgaeAtA2(myIntakeLow, myElevatorWrist, false),
        CoralIntakePositionCmd(myIntakeLow, myElevatorWrist),
        AlgaeToNetCmd(myIntakeLow, myElevatorWrist, false),
        CoralIntakePositionCmd(myIntakeLow, myElevatorWrist),
        AlgaeCradle(myIntakeLow, myElevatorWrist),
        CoralIntakePositionCmd(myIntakeLow, myElevatorWrist),
        // home L2
        Commands.waitSeconds(0.3),
        CoralL2Cmd(myIntakeLow, myElevatorWrist),
        CoralL3Cmd(myIntakeLow, myElevatorWrist),
        CoralL2Cmd(myIntakeLow, myElevatorWrist),
        CoralL4Cmd(myIntakeLow, myElevatorWrist),
        CoralL2Cmd(myIntakeLow, myElevatorWrist),
        CoralL1Cmd(myIntakeLow, myElevatorWrist),
        CoralL2Cmd(myIntakeLow, myElevatorWrist),
        AlgaeToP1(myIntakeLow, myElevatorWrist, false),
        CoralL2Cmd(myIntakeLow, myElevatorWrist),
        AlgaeAtA1(myIntakeLow, myElevatorWrist, false),
        CoralL2Cmd(myIntakeLow, myElevatorWrist),
        AlgaeAtA2(myIntakeLow, myElevatorWrist, false),
        CoralL2Cmd(myIntakeLow, myElevatorWrist),
        AlgaeToNetCmd(myIntakeLow, myElevatorWrist, false),
        CoralL2Cmd(myIntakeLow, myElevatorWrist),
        AlgaeCradle(myIntakeLow, myElevatorWrist),
        CoralL2Cmd(myIntakeLow, myElevatorWrist),
        // home L3
        Commands.waitSeconds(0.3),
        CoralL3Cmd(myIntakeLow, myElevatorWrist),
        CoralL4Cmd(myIntakeLow, myElevatorWrist),
        CoralL3Cmd(myIntakeLow, myElevatorWrist),
        CoralL1Cmd(myIntakeLow, myElevatorWrist),
        CoralL3Cmd(myIntakeLow, myElevatorWrist),
        AlgaeToP1(myIntakeLow, myElevatorWrist, false),
        CoralL3Cmd(myIntakeLow, myElevatorWrist),
        AlgaeAtA1(myIntakeLow, myElevatorWrist, false),
        CoralL3Cmd(myIntakeLow, myElevatorWrist),
        AlgaeAtA2(myIntakeLow, myElevatorWrist, false),
        CoralL3Cmd(myIntakeLow, myElevatorWrist),
        AlgaeToNetCmd(myIntakeLow, myElevatorWrist, false),
        CoralL3Cmd(myIntakeLow, myElevatorWrist),
        AlgaeCradle(myIntakeLow, myElevatorWrist),
        CoralL3Cmd(myIntakeLow, myElevatorWrist),
        // home L4
        Commands.waitSeconds(0.3),
        CoralL4Cmd(myIntakeLow, myElevatorWrist),
        CoralL1Cmd(myIntakeLow, myElevatorWrist),
        CoralL4Cmd(myIntakeLow, myElevatorWrist),
        AlgaeToP1(myIntakeLow, myElevatorWrist, false),
        CoralL4Cmd(myIntakeLow, myElevatorWrist),
        AlgaeAtA1(myIntakeLow, myElevatorWrist, false),
        CoralL4Cmd(myIntakeLow, myElevatorWrist),
        AlgaeAtA2(myIntakeLow, myElevatorWrist, false),
        CoralL4Cmd(myIntakeLow, myElevatorWrist),
        AlgaeToNetCmd(myIntakeLow, myElevatorWrist, false),
        CoralL4Cmd(myIntakeLow, myElevatorWrist),
        AlgaeCradle(myIntakeLow, myElevatorWrist),
        CoralL4Cmd(myIntakeLow, myElevatorWrist),
        // home L1
        Commands.waitSeconds(0.3),
        CoralL1Cmd(myIntakeLow, myElevatorWrist),
        AlgaeToP1(myIntakeLow, myElevatorWrist, false),
        CoralL1Cmd(myIntakeLow, myElevatorWrist),
        AlgaeAtA1(myIntakeLow, myElevatorWrist, false),
        CoralL1Cmd(myIntakeLow, myElevatorWrist),
        AlgaeAtA2(myIntakeLow, myElevatorWrist, false),
        CoralL1Cmd(myIntakeLow, myElevatorWrist),
        AlgaeToNetCmd(myIntakeLow, myElevatorWrist, false),
        CoralL1Cmd(myIntakeLow, myElevatorWrist),
        AlgaeCradle(myIntakeLow, myElevatorWrist),
        CoralL1Cmd(myIntakeLow, myElevatorWrist),
        // home P1
        Commands.waitSeconds(0.3),
        AlgaeToP1(myIntakeLow, myElevatorWrist, false),
        AlgaeAtA1(myIntakeLow, myElevatorWrist, false),
        AlgaeToP1(myIntakeLow, myElevatorWrist, false),
        AlgaeAtA2(myIntakeLow, myElevatorWrist, false),
        AlgaeToP1(myIntakeLow, myElevatorWrist, false),
        AlgaeToNetCmd(myIntakeLow, myElevatorWrist, false),
        AlgaeToP1(myIntakeLow, myElevatorWrist, false),
        AlgaeCradle(myIntakeLow, myElevatorWrist),
        AlgaeToP1(myIntakeLow, myElevatorWrist, false),
        // home A1
        Commands.waitSeconds(0.3),
        AlgaeAtA1(myIntakeLow, myElevatorWrist, false),
        AlgaeAtA2(myIntakeLow, myElevatorWrist, false),
        AlgaeAtA1(myIntakeLow, myElevatorWrist, false),
        AlgaeToNetCmd(myIntakeLow, myElevatorWrist, false),
        AlgaeAtA1(myIntakeLow, myElevatorWrist, false),
        AlgaeCradle(myIntakeLow, myElevatorWrist),
        AlgaeAtA1(myIntakeLow, myElevatorWrist, false),
        // home A2
        Commands.waitSeconds(0.3),
        AlgaeAtA2(myIntakeLow, myElevatorWrist, false),
        AlgaeToNetCmd(myIntakeLow, myElevatorWrist, false),
        AlgaeAtA2(myIntakeLow, myElevatorWrist, false),
        AlgaeCradle(myIntakeLow, myElevatorWrist),
        AlgaeAtA2(myIntakeLow, myElevatorWrist, false),
        // home net
        Commands.waitSeconds(0.3),
        AlgaeToNetCmd(myIntakeLow, myElevatorWrist, false),
        AlgaeCradle(myIntakeLow, myElevatorWrist),
        AlgaeToNetCmd(myIntakeLow, myElevatorWrist, false),
        // return to zero
        Commands.waitSeconds(0.3),
        ElevatorWristZeroCmd(myIntakeLow, myElevatorWrist));
  }
}
