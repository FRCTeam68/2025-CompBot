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
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorWristSubsystem;
import frc.robot.subsystems.RangeSensorSubsystem;
import frc.robot.subsystems.ShotVisualizer;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.rollers.RollerSystem;
import java.util.Set;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ManipulatorCommands {
  @Getter @AutoLogOutput private static ScoringPosition scoringPosition = ScoringPosition.CoralL2_4;
  @Getter @Setter @AutoLogOutput private static boolean havePiece = false;
  @Getter @AutoLogOutput private static boolean indexing = false;

  private ManipulatorCommands(Lights LED) {}

  public static enum ScoringPosition {
    Algae,

    AlgaeNet,

    CoralL1,

    CoralL2_4
  }

  public static Command intakeCmd(
      RollerSystem myIntake,
      RollerSystem myIntakeLow,
      ElevatorWristSubsystem myElevatorWrist,
      RangeSensorSubsystem intake_sensor,
      Lights LED) {
    return new DeferredCommand(
        () -> {
          // initialization
          Command command;
          Command initialize =
              Commands.runOnce(() -> LED.setBandAnimation(LEDColor.BLUE, LEDSegment.ALL));
          Command ledHaveObject =
              Commands.runOnce(() -> LED.setSolidColor(LEDColor.BLUE, LEDSegment.ALL));
          Command finalize =
              Commands.parallel(
                  Commands.runOnce(() -> indexing = false),
                  // Commands.runOnce(() -> safeToMove = true),
                  Commands.runOnce(() -> havePiece = true));
          command = Commands.none();
          if (Constants.currentMode != Mode.SIM) {
            if (scoringPosition == ScoringPosition.Algae
                || scoringPosition == ScoringPosition.AlgaeNet) {
              ///// INTAKE ALGAE /////
              command =
                  Commands.sequence(
                      Commands.runOnce(() -> havePiece = false),
                      Commands.runOnce(
                          () ->
                              Logger.recordOutput("Manipulator/IntakeShooterState", "IntakeAlgae")),
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
                        CoralIntakePositionCmd(myElevatorWrist),
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
                                Logger.recordOutput(
                                    "Manipulator/IntakeShooterState", "IntakeCoral")),
                        CoralIntakePositionCmd(myElevatorWrist),
                        myIntake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_SPEED),
                        Commands.waitUntil(() -> intake_sensor.isDetected()),
                        Commands.runOnce(() -> indexing = true),
                        Commands.waitSeconds(.03),
                        myIntake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_INDEX_SPEED),
                        Commands.waitUntil(() -> intake_sensor.isDetected() == false),
                        myIntake.setSpeedCmd(
                            Constants.INTAKE_SHOOTER.CORAL_INTAKE_INDEX_SPEED * -1),
                        Commands.waitUntil(() -> intake_sensor.isDetected()),
                        Commands.runOnce(
                            () ->
                                myIntake.setPosition(
                                    myIntake.getPosition()
                                        - Constants.INTAKE_SHOOTER.CORAL_INTAKE_INDEX_REVERSE)),
                        ledHaveObject);
              }
            }
          } else {
            if (scoringPosition == ScoringPosition.Algae
                || scoringPosition == ScoringPosition.AlgaeNet) {
              command = Commands.waitSeconds(1);
            } else {
              command =
                  Commands.sequence(
                      CoralIntakePositionCmd(myElevatorWrist), Commands.waitSeconds(1));
            }
          }
          // execute sequence
          return initialize.andThen(command).andThen(finalize);
        },
        Set.of(myIntake, myIntakeLow));
  }

  public static Command shootCmd(
      RollerSystem myIntake,
      RollerSystem myIntakeLow,
      ElevatorWristSubsystem myElevatorWrist,
      Lights LED) {
    return new DeferredCommand(
        () -> {
          // initialization
          Command command;
          Command ledShooting =
              Commands.sequence(
                  // need to set false before setting LED green so elevatorWrist periodic does not
                  // turn back to red during the shoot command
                  Commands.runOnce(() -> havePiece = false),
                  Commands.runOnce(() -> indexing = false),
                  Commands.runOnce(() -> myElevatorWrist.setLookingToShoot(false)),
                  Commands.runOnce(() -> LED.setSolidColor(LEDColor.GREEN, LEDSegment.ALL)));
          Command afterShot =
              Commands.parallel(
                  Commands.runOnce(() -> LED.disableLEDs(LEDSegment.ALL)),
                  myIntake.setSpeedCmd(0),
                  myIntakeLow.setSpeedCmd(0));
          command = Commands.none();
          if (scoringPosition == ScoringPosition.Algae) {
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
          } else if (scoringPosition == ScoringPosition.AlgaeNet) {
            ///// SHOOT ALGAE NET /////
            command =
                Commands.parallel(
                    Commands.runOnce(
                        () -> Logger.recordOutput("Manipulator/ElevatorWristState", "ShootNet")),
                    ShotVisualizer.shootParabula(),
                    myElevatorWrist.setPositionCmd(
                        Constants.ELEVATOR.SHOOTNET, Constants.WRIST.SHOOTNET, 1),
                    Commands.sequence(
                        Commands.waitSeconds(Constants.INTAKE_SHOOTER.ALGAE_NET_SHOOT_DELAY),
                        myIntake.setSpeedCmd(Constants.INTAKE_SHOOTER.ALGAE_NET_SHOOT_SPEED),
                        myIntakeLow.setSpeedCmd(Constants.INTAKE_SHOOTER_LOW.ALGAE_NET_SHOOT_SPEED),
                        Commands.waitSeconds(Constants.INTAKE_SHOOTER.ALGAE_SHOOT_TIMEOUT)));
          } else if (scoringPosition == ScoringPosition.CoralL1) {
            ///// SHOOT CORAL L1 /////
            command =
                Commands.sequence(
                    Commands.runOnce(
                        () ->
                            Logger.recordOutput("Manipulator/IntakeShooterState", "ShootCoralL1")),
                    myIntake.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_L1_SHOOT_SPEED),
                    Commands.waitSeconds(Constants.INTAKE_SHOOTER.CORAL_L1_SHOOT_TIMEOUT));
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
        Set.of(myIntake, myIntakeLow));
  }

  public static Command CoralL4Cmd(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.parallel(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.CoralL2_4),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "L4")),
        Commands.either(
            Commands.sequence(
                myElevatorWrist.setPositionCmd(Constants.ELEVATOR.L4, Constants.WRIST.L4),
                Commands.runOnce(() -> myElevatorWrist.setLookingToShoot(true))),
            Commands.none(),
            () -> {
              return indexing || havePiece || RobotContainer.isM_overideMode();
            }));
  }

  public static Command CoralL3Cmd(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.CoralL2_4),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "L3")),
        Commands.either(
            Commands.sequence(
                myElevatorWrist.setPositionCmd(Constants.ELEVATOR.L3, Constants.WRIST.L3),
                Commands.runOnce(() -> myElevatorWrist.setLookingToShoot(true))),
            Commands.none(),
            () -> {
              return indexing || havePiece || RobotContainer.isM_overideMode();
            }));
  }

  public static Command CoralL2Cmd(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.CoralL2_4),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "L2")),
        Commands.either(
            Commands.sequence(
                myElevatorWrist.setPositionCmd(Constants.ELEVATOR.L2, Constants.WRIST.L2),
                Commands.runOnce(() -> myElevatorWrist.setLookingToShoot(true))),
            Commands.none(),
            () -> {
              return indexing || havePiece || RobotContainer.isM_overideMode();
            }));
  }

  public static Command CoralL1Cmd(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.CoralL1),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "L1")),
        Commands.either(
            myElevatorWrist.setPositionCmd(Constants.ELEVATOR.L1, Constants.WRIST.L1, 1),
            Commands.none(),
            () -> {
              return indexing || havePiece || RobotContainer.isM_overideMode();
            }));
  }

  public static Command CoralIntakePositionCmd(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.CoralL2_4),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "INTAKE")),
        myElevatorWrist.setPositionCmd(Constants.ELEVATOR.INTAKE, Constants.WRIST.INTAKE));
  }

  public static Command AlgaeToNetCmd(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.AlgaeNet),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "NET")),
        myElevatorWrist.setPositionCmd(Constants.ELEVATOR.PRENET, Constants.WRIST.PRENET, 1));
  }

  public static Command AlgaeToP1(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.Algae),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "P1")),
        myElevatorWrist.setPositionCmd(Constants.ELEVATOR.P1, Constants.WRIST.P1));
  }

  public static Command AlgaeAtA2(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.Algae),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "A2")),
        myElevatorWrist.setPositionCmd(Constants.ELEVATOR.A2, Constants.WRIST.A2));
  }

  public static Command AlgaeAtA1(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.Algae),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "A1")),
        myElevatorWrist.setPositionCmd(Constants.ELEVATOR.A1, Constants.WRIST.A1));
  }

  public static Command AlgaeCradle(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.Algae),
        Commands.runOnce(
            () -> Logger.recordOutput("Manipulator/ElevatorWristState", "AlgaeCradle")),
        myElevatorWrist.setPositionCmd(Constants.ELEVATOR.MIN_POSITION, Constants.WRIST.SAFE),
        myElevatorWrist.setPositionCmd(Constants.ELEVATOR.MIN_POSITION, Constants.WRIST.CRADLE));
  }

  public static Command DeployClimberCmd(RollerSystem myClimber, Lights LED) {
    return Commands.sequence(
        Commands.runOnce(() -> LED.setSingleFadeAnimation(LEDColor.RED, LEDSegment.ALL)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "deploying")),
        Commands.runOnce(() -> myClimber.setPosition(Constants.CLIMBER.DEPLOY), myClimber),
        Commands.waitUntil(() -> myClimber.atPosition()),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "deployed")),
        Commands.runOnce(() -> LED.setSolidColor(LEDColor.RED, LEDSegment.ALL)));
  }

  public static Command RetractClimberCmd(RollerSystem myClimber, Lights LED) {
    return Commands.sequence(
        Commands.runOnce(() -> LED.setSingleFadeAnimation(LEDColor.GREEN, LEDSegment.ALL)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "climbing")),
        Commands.runOnce(() -> myClimber.setPosition(Constants.CLIMBER.RETRACT), myClimber),
        Commands.waitUntil(() -> myClimber.atPosition()),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "CLIMBED")),
        Commands.runOnce(() -> LED.setRainbowAnimation(LEDSegment.ALL)));
  }

  public static Command climberToZeroCmd(RollerSystem myClimber, Lights LED) {
    return Commands.sequence(
        Commands.runOnce(() -> LED.setSingleFadeAnimation(LEDColor.RED, LEDSegment.ALL)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "to zero")),
        Commands.runOnce(() -> myClimber.setPosition(0), myClimber),
        Commands.waitUntil(() -> myClimber.atPosition()),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "at zero")),
        Commands.runOnce(() -> LED.setRainbowAnimation(LEDSegment.ALL)));
  }

  // NO CHECKS
  public static Command BumpClimberCmd(RollerSystem myClimber, double bumpValue) {
    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "bumping")),
        Commands.runOnce(
            () -> myClimber.setPosition(myClimber.getPosition() + bumpValue), myClimber),
        Commands.waitUntil(() -> myClimber.atPosition()),
        Commands.runOnce(() -> myClimber.zero()));
  }

  public static Command ElevatorWristZeroCmd(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.CoralL2_4),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "ZERO")),
        myElevatorWrist.setPositionCmd(
            Constants.ELEVATOR.MIN_POSITION, Constants.WRIST.MIN_POSITION));
  }

  public static Command FunctionalTest(
      RollerSystem myIntake,
      RollerSystem myIntakeLow,
      ElevatorWristSubsystem myElevatorWrist,
      RangeSensorSubsystem intake_sensor,
      RollerSystem myClimber,
      Lights LED) {
    return Commands.sequence(
        CoralIntakePositionCmd(myElevatorWrist),
        intakeCmd(myIntake, myIntakeLow, myElevatorWrist, intake_sensor, LED),
        CoralL4Cmd(myElevatorWrist),
        Commands.waitSeconds(0.5),
        CoralL1Cmd(myElevatorWrist),
        Commands.waitSeconds(0.5),
        shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED),
        Commands.waitSeconds(1),
        AlgaeToP1(myElevatorWrist),
        intakeCmd(myIntake, myIntakeLow, myElevatorWrist, intake_sensor, LED),
        AlgaeToNetCmd(myElevatorWrist),
        Commands.waitSeconds(0.5),
        AlgaeToP1(myElevatorWrist),
        Commands.waitSeconds(0.5),
        shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED),
        Commands.waitSeconds(1),
        ElevatorWristZeroCmd(myElevatorWrist),
        Commands.waitSeconds(0.5),
        DeployClimberCmd(myClimber, LED),
        Commands.waitSeconds(0.5),
        climberToZeroCmd(myClimber, LED));
  }

  public static Command TestElevatorWristSequencing(ElevatorWristSubsystem myElevatorWrist) {
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
        AlgaeToP1(myElevatorWrist),
        CoralIntakePositionCmd(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist),
        CoralIntakePositionCmd(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist),
        CoralIntakePositionCmd(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist),
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
        AlgaeToP1(myElevatorWrist),
        CoralL2Cmd(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist),
        CoralL2Cmd(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist),
        CoralL2Cmd(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist),
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
        AlgaeToP1(myElevatorWrist),
        CoralL3Cmd(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist),
        CoralL3Cmd(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist),
        CoralL3Cmd(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist),
        CoralL3Cmd(myElevatorWrist),
        AlgaeCradle(myElevatorWrist),
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
        AlgaeCradle(myElevatorWrist),
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
        AlgaeCradle(myElevatorWrist),
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
        AlgaeCradle(myElevatorWrist),
        AlgaeToP1(myElevatorWrist),
        // home A1
        Commands.waitSeconds(0.3),
        AlgaeAtA1(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist),
        AlgaeCradle(myElevatorWrist),
        AlgaeAtA1(myElevatorWrist),
        // home A2
        Commands.waitSeconds(0.3),
        AlgaeAtA2(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist),
        AlgaeCradle(myElevatorWrist),
        AlgaeAtA2(myElevatorWrist),
        // home net
        Commands.waitSeconds(0.3),
        AlgaeToNetCmd(myElevatorWrist),
        AlgaeCradle(myElevatorWrist),
        AlgaeToNetCmd(myElevatorWrist),
        // return to zero
        Commands.waitSeconds(0.3),
        ElevatorWristZeroCmd(myElevatorWrist));
  }
}
