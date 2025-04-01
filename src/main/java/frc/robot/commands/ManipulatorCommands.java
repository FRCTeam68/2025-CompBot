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
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorWristSubSystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import frc.robot.subsystems.RangeSensorSubSystem;
import frc.robot.subsystems.rollers.RollerSystem;
import java.util.Set;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ManipulatorCommands {

  @Getter @AutoLogOutput private static boolean indexing = false;
  @Getter @AutoLogOutput private static boolean safeToMove = false;
  @Getter @AutoLogOutput private static boolean havePiece = false;

  private static final ElevatorWristSubSystem elevatorWrist = RobotContainer.elevatorWrist;
  private static final RollerSystem climber = RobotContainer.climber;
  private static final RollerSystem intake = RobotContainer.intakeShooter;
  private static final RollerSystem intakeLow = RobotContainer.intakeShooterLow;
  private static final RangeSensorSubSystem intakeCoralSensor = RobotContainer.intakeCoralSensor;

  private ManipulatorCommands() {}

  public static void setHavePiece(boolean state) {
    safeToMove = state;
  }

  public static Command intakeCmd() {
    return new DeferredCommand(
        () -> {
          // initialization
          Command command;
          Command initialize =
              Commands.parallel(
                  Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.blue, 4)),
                  Commands.runOnce(() -> indexing = false),
                  Commands.runOnce(() -> safeToMove = false),
                  Commands.runOnce(() -> havePiece = false));
          Command ledHaveObject =
              Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.blue));
          Command finalize =
              Commands.sequence(
                  Commands.runOnce(() -> indexing = false),
                  Commands.runOnce(() -> safeToMove = true),
                  Commands.runOnce(() -> havePiece = true));
          command = Commands.none();

          if (Constants.WRIST.POSITION_SCORING_ELEMENT == "Algae"
              || Constants.WRIST.POSITION_SCORING_ELEMENT == "AlgaeNet") {
            ///// INTAKE ALGAE /////
            command =
                Commands.sequence(
                    Commands.runOnce(
                        () -> Logger.recordOutput("Manipulator/IntakeShooterState", "IntakeAlgae")),
                    intake.setSpeedCmd(Constants.INTAKE_SHOOTER.ALGAE_INTAKE_SPEED),
                    intakeLow.setSpeedCmd(Constants.INTAKE_SHOOTER_LOW.ALGAE_INTAKE_SPEED),
                    Commands.waitUntil(() -> intake.hasPiece()),
                    ledHaveObject,
                    Commands.runOnce(() -> safeToMove = true),
                    intake.setSpeedCmd(Constants.INTAKE_SHOOTER.ALGAE_HOLD_SPEED),
                    intakeLow.setSpeedCmd(Constants.INTAKE_SHOOTER_LOW.ALGAE_HOLD_SPEED));
          } else {
            if (intakeCoralSensor.havePiece()) {
              ///// ALREADY HAVE CORAL /////
              command =
                  Commands.parallel(
                      Commands.runOnce(
                          () ->
                              Logger.recordOutput(
                                  "Manipulator/IntakeShooterState", "AlreadyHasCoral")),
                      ledHaveObject);
            } else {
              ///// INTAKE CORAL /////
              command =
                  Commands.sequence(
                      Commands.runOnce(
                          () ->
                              Logger.recordOutput("Manipulator/IntakeShooterState", "IntakeCoral")),
                      CoralIntakePositionCmd(),
                      intake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_SPEED),
                      Commands.waitUntil(() -> intakeCoralSensor.havePiece()),
                      ledHaveObject,
                      Commands.runOnce(() -> indexing = true),
                      Commands.waitSeconds(.03),
                      intake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_INDEX_SPEED),
                      Commands.waitUntil(() -> intakeCoralSensor.havePiece() == false),
                      intake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_INDEX_SPEED * -1),
                      Commands.runOnce(() -> safeToMove = true),
                      Commands.waitUntil(() -> intakeCoralSensor.havePiece()),
                      Commands.runOnce(
                          () ->
                              intake.setPosition(
                                  intake.getPosition()
                                      - Constants.INTAKE_SHOOTER.CORAL_INTAKE_INDEX_REVERSE)),
                      Commands.waitUntil(() -> (intake.atPosition() || intake.getSpeed() < 0.05)));
            }
          }
          // execute sequence
          return initialize.andThen(command).andThen(finalize);
        },
        Set.of(intake, intakeLow));
  }

  public static Command shootCmd() {
    return new DeferredCommand(
        () -> {
          // initialization
          Command command;
          Command initialize =
              Commands.parallel(
                  Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.red)),
                  Commands.runOnce(() -> indexing = false),
                  Commands.runOnce(() -> safeToMove = false),
                  Commands.runOnce(() -> havePiece = false));
          Command finalize =
              Commands.parallel(
                  Commands.runOnce(() -> LEDSegment.all.disableLEDs()),
                  intake.setSpeedCmd(0),
                  intakeLow.setSpeedCmd(0));
          command = Commands.none();
          if (Constants.WRIST.POSITION_SCORING_ELEMENT == "Algae") {
            ///// SHOOT ALGAE PROCESSOR /////
            command =
                Commands.sequence(
                    Commands.runOnce(
                        () ->
                            Logger.recordOutput(
                                "Manipulator/IntakeShooterState", "ShootProcessor")),
                    intake.setSpeedCmd(Constants.INTAKE_SHOOTER.ALGAE_SHOOT_SPEED),
                    intakeLow.setSpeedCmd(Constants.INTAKE_SHOOTER_LOW.ALGAE_SHOOT_SPEED),
                    Commands.waitSeconds(Constants.INTAKE_SHOOTER.ALGAE_SHOOT_TIMEOUT));
          } else if (Constants.WRIST.POSITION_SCORING_ELEMENT == "AlgaeNet") {
            ///// SHOOT ALGAE NET /////
            command =
                Commands.parallel(
                    Commands.runOnce(
                        () -> Logger.recordOutput("Manipulator/ElevatorWristState", "ShootNet")),
                    AlgaeShootNetCmd(),
                    Commands.sequence(
                        Commands.waitSeconds(Constants.INTAKE_SHOOTER.ALGAE_NET_SHOOT_DELAY),
                        intake.setSpeedCmd(Constants.INTAKE_SHOOTER.ALGAE_NET_SHOOT_SPEED),
                        intakeLow.setSpeedCmd(Constants.INTAKE_SHOOTER_LOW.ALGAE_NET_SHOOT_SPEED),
                        Commands.waitSeconds(Constants.INTAKE_SHOOTER.ALGAE_SHOOT_TIMEOUT)));
          } else if (Constants.WRIST.POSITION_SCORING_ELEMENT == "CoralL1") {
            ///// SHOOT CORAL L1 /////
            command =
                Commands.sequence(
                    Commands.runOnce(
                        () ->
                            Logger.recordOutput("Manipulator/IntakeShooterState", "ShootCoralL1")),
                    intake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_L1_SHOOT_SPEED),
                    Commands.waitSeconds(Constants.INTAKE_SHOOTER.CORAL_L1_SHOOT_TIMEOUT));
          } else {
            ///// SHOOT CORAL L2 - L4 /////
            command =
                Commands.sequence(
                    Commands.runOnce(
                        () -> Logger.recordOutput("Manipulator/IntakeShooterState", "ShootCoral")),
                    intake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
                    Commands.waitSeconds(Constants.INTAKE_SHOOTER.CORAL_SHOOT_TIMEOUT));
          }
          // execute sequence
          return initialize.andThen(command).andThen(finalize);
        },
        Set.of(intake, intakeLow));
  }

  public static Command CoralL4Cmd() {
    return Commands.either(
        Commands.parallel(
            Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "Coral"),
            Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "L4")),
            elevatorWrist.setPositionCmdNew(Constants.ELEVATOR.L4, Constants.WRIST.L4)),
        Commands.runOnce(
            () -> Logger.recordOutput("Manipulator/ElevatorWristState", "CANNOT MOVE L4")),
        () -> {
          return havePiece || safeToMove || RobotContainer.m_overideMode;
        });
  }

  public static Command CoralL3Cmd() {
    return Commands.either(
        Commands.parallel(
            Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "Coral"),
            Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "L3")),
            elevatorWrist.setPositionCmdNew(Constants.ELEVATOR.L3, Constants.WRIST.L3)),
        Commands.runOnce(
            () -> Logger.recordOutput("Manipulator/ElevatorWristState", "CANNOT MOVE L3")),
        () -> {
          return havePiece || safeToMove || RobotContainer.m_overideMode;
        });
  }

  public static Command CoralL2Cmd() {
    return Commands.either(
        Commands.parallel(
            Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "Coral"),
            Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "L2")),
            elevatorWrist.setPositionCmdNew(Constants.ELEVATOR.L2, Constants.WRIST.L2)),
        Commands.runOnce(
            () -> Logger.recordOutput("Manipulator/ElevatorWristState", "CANNOT MOVE L2")),
        () -> {
          return havePiece || safeToMove || RobotContainer.m_overideMode;
        });
  }

  public static Command CoralL1Cmd() {
    return Commands.either(
        Commands.parallel(
            Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "CoralL1"),
            Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "L1")),
            elevatorWrist.setPositionCmdNew(Constants.ELEVATOR.L1, Constants.WRIST.L1, 1)),
        Commands.runOnce(
            () -> Logger.recordOutput("Manipulator/ElevatorWristState", "CANNOT MOVE L1")),
        () -> {
          return havePiece || safeToMove || RobotContainer.m_overideMode;
        });
  }

  public static Command CoralIntakePositionCmd() {
    return Commands.parallel(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "Coral"),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "INTAKE")),
        elevatorWrist.setPositionCmdNew(Constants.ELEVATOR.INTAKE, Constants.WRIST.INTAKE));
  }

  public static Command AlgaeToNetCmd() {
    return Commands.parallel(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "AlgaeNet"),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "NET")),
        elevatorWrist.setPositionCmdNew(Constants.ELEVATOR.PRENET, Constants.WRIST.PRENET));
  }

  public static Command AlgaeShootNetCmd() {
    return Commands.parallel(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "AlgaeNet"),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "SHOOTNET")),
        elevatorWrist.setPositionCmdNew(Constants.ELEVATOR.SHOOTNET, Constants.WRIST.SHOOTNET, 1));
  }

  public static Command AlgaeToP1() {
    return Commands.parallel(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "Algae"),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "P1")),
        elevatorWrist.setPositionCmdNew(Constants.ELEVATOR.P1, Constants.WRIST.P1));
  }

  public static Command AlgaeAtA2() {
    return Commands.parallel(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "Algae"),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "A2")),
        elevatorWrist.setPositionCmdNew(Constants.ELEVATOR.A2, Constants.WRIST.A2));
  }

  public static Command AlgaeAtA1() {
    return Commands.parallel(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "Algae"),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "A1")),
        elevatorWrist.setPositionCmdNew(Constants.ELEVATOR.A1, Constants.WRIST.A1));
  }

  public static Command AlgaeCradle() {
    return Commands.parallel(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "Algae"),
        Commands.runOnce(
            () -> Logger.recordOutput("Manipulator/ElevatorWristState", "AlgaeCradle")),
        elevatorWrist
            .setPositionCmdNew(Constants.ELEVATOR.MIN_POSITION, Constants.WRIST.SAFE)
            .andThen(
                elevatorWrist.setPositionCmdNew(
                    Constants.ELEVATOR.MIN_POSITION, Constants.WRIST.CRADLE)));
  }

  public static Command ElevatorWristZeroCmd() {
    return Commands.sequence(
        Commands.runOnce(() -> Constants.WRIST.POSITION_SCORING_ELEMENT = "Null"),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "ZERO")),
        elevatorWrist.setPositionCmdNew(
            Constants.ELEVATOR.MIN_POSITION, Constants.WRIST.MIN_POSITION));
  }

  public static Command DeployClimberCmd() {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setFadeAnimation(LightsSubsystem.red, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "deploying")),
        Commands.runOnce(() -> climber.setPosition(Constants.CLIMBER.DEPLOY), climber),
        Commands.waitUntil(() -> climber.atPosition()),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "deployed")),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.red)));
  }

  public static Command RetractClimberCmd() {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setFadeAnimation(LightsSubsystem.red, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "climbing")),
        Commands.runOnce(() -> climber.setPosition(Constants.CLIMBER.RETRACT), climber),
        Commands.waitUntil(() -> climber.atPosition()),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "CLIMBED")),
        Commands.runOnce(() -> LEDSegment.all.setRainbowAnimation(4)));
  }

  public static Command climberToZeroCmd() {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setFadeAnimation(LightsSubsystem.red, 4)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "to zero")),
        Commands.runOnce(() -> climber.setPosition(0), climber),
        Commands.waitUntil(() -> climber.atPosition()),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "at zero")),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.red)));
  }

  // NO CHECKS
  public static Command BumpClimberCmd(double bumpValue) {
    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "bumping")),
        Commands.runOnce(() -> climber.setPosition(climber.getPosition() + bumpValue), climber),
        Commands.waitUntil(() -> climber.atPosition()),
        Commands.runOnce(() -> climber.zero()));
  }

  public static Command FunctionalTest() {
    return Commands.sequence(
        CoralIntakePositionCmd(),
        intakeCmd(),
        CoralL4Cmd(),
        Commands.waitSeconds(0.5),
        CoralL1Cmd(),
        Commands.waitSeconds(0.5),
        shootCmd(),
        Commands.waitSeconds(1),
        AlgaeToP1(),
        intakeCmd(),
        AlgaeToNetCmd(),
        Commands.waitSeconds(0.5),
        AlgaeToP1(),
        Commands.waitSeconds(0.5),
        shootCmd(),
        Commands.waitSeconds(1),
        ElevatorWristZeroCmd(),
        Commands.waitSeconds(0.5),
        DeployClimberCmd(),
        Commands.waitSeconds(0.5),
        RetractClimberCmd(),
        climberToZeroCmd());
  }

  public static Command TestElevatorWristSequencing() {
    return Commands.sequence(
        // home intake
        CoralIntakePositionCmd(),
        CoralL2Cmd(),
        CoralIntakePositionCmd(),
        CoralL3Cmd(),
        CoralIntakePositionCmd(),
        CoralL4Cmd(),
        CoralIntakePositionCmd(),
        CoralL1Cmd(),
        CoralIntakePositionCmd(),
        AlgaeToP1(),
        CoralIntakePositionCmd(),
        AlgaeAtA1(),
        CoralIntakePositionCmd(),
        AlgaeAtA2(),
        CoralIntakePositionCmd(),
        AlgaeToNetCmd(),
        CoralIntakePositionCmd(),
        AlgaeCradle(),
        CoralIntakePositionCmd(),
        // home L2
        Commands.waitSeconds(0.3),
        CoralL2Cmd(),
        CoralL3Cmd(),
        CoralL2Cmd(),
        CoralL4Cmd(),
        CoralL2Cmd(),
        CoralL1Cmd(),
        CoralL2Cmd(),
        AlgaeToP1(),
        CoralL2Cmd(),
        AlgaeAtA1(),
        CoralL2Cmd(),
        AlgaeAtA2(),
        CoralL2Cmd(),
        AlgaeToNetCmd(),
        CoralL2Cmd(),
        AlgaeCradle(),
        CoralL2Cmd(),
        // home L3
        Commands.waitSeconds(0.3),
        CoralL3Cmd(),
        CoralL4Cmd(),
        CoralL3Cmd(),
        CoralL1Cmd(),
        CoralL3Cmd(),
        AlgaeToP1(),
        CoralL3Cmd(),
        AlgaeAtA1(),
        CoralL3Cmd(),
        AlgaeAtA2(),
        CoralL3Cmd(),
        AlgaeToNetCmd(),
        CoralL3Cmd(),
        AlgaeCradle(),
        CoralL3Cmd(),
        // home L4
        Commands.waitSeconds(0.3),
        CoralL4Cmd(),
        CoralL1Cmd(),
        CoralL4Cmd(),
        AlgaeToP1(),
        CoralL4Cmd(),
        AlgaeAtA1(),
        CoralL4Cmd(),
        AlgaeAtA2(),
        CoralL4Cmd(),
        AlgaeToNetCmd(),
        CoralL4Cmd(),
        AlgaeCradle(),
        CoralL4Cmd(),
        // home L1
        Commands.waitSeconds(0.3),
        CoralL1Cmd(),
        AlgaeToP1(),
        CoralL1Cmd(),
        AlgaeAtA1(),
        CoralL1Cmd(),
        AlgaeAtA2(),
        CoralL1Cmd(),
        AlgaeToNetCmd(),
        CoralL1Cmd(),
        AlgaeCradle(),
        CoralL1Cmd(),
        // home P1
        Commands.waitSeconds(0.3),
        AlgaeToP1(),
        AlgaeAtA1(),
        AlgaeToP1(),
        AlgaeAtA2(),
        AlgaeToP1(),
        AlgaeToNetCmd(),
        AlgaeToP1(),
        AlgaeCradle(),
        AlgaeToP1(),
        // home A1
        Commands.waitSeconds(0.3),
        AlgaeAtA1(),
        AlgaeAtA2(),
        AlgaeAtA1(),
        AlgaeToNetCmd(),
        AlgaeAtA1(),
        AlgaeCradle(),
        AlgaeAtA1(),
        // home A2
        Commands.waitSeconds(0.3),
        AlgaeAtA2(),
        AlgaeToNetCmd(),
        AlgaeAtA2(),
        AlgaeCradle(),
        AlgaeAtA2(),
        // home net
        Commands.waitSeconds(0.3),
        AlgaeToNetCmd(),
        AlgaeCradle(),
        AlgaeToNetCmd(),
        // return to zero
        Commands.waitSeconds(0.3),
        ElevatorWristZeroCmd());
  }
}
