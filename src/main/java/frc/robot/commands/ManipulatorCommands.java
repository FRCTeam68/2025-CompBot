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

import com.ctre.phoenix6.signals.AnimationDirectionValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants;
import frc.robot.Constants.LEDColor;
import frc.robot.Constants.LEDSegment;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShotVisualizer;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.sensors.RangeSensor;
import frc.robot.subsystems.superstructure.ElevatorWristSubsystem;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ManipulatorCommands {
  @Getter @AutoLogOutput private static ScoringPosition scoringPosition = ScoringPosition.CoralL2_4;

  @Getter @Setter @AutoLogOutput private static boolean havePiece = false;

  @Getter @AutoLogOutput private static boolean indexing = false;

  public ManipulatorCommands() {}

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
      RangeSensor intake_sensor,
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

          if (scoringPosition == ScoringPosition.Algae
              || scoringPosition == ScoringPosition.AlgaeNet) {
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
                              Logger.recordOutput("Manipulator/IntakeShooterState", "IntakeCoral")),
                      CoralIntakePositionCmd(myElevatorWrist),
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
          return initialize.andThen(command).andThen(finalize);
        },
        Set.of(myIntake, myIntakeLow, myElevatorWrist));
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
                    myElevatorWrist.setPositionCmd(SuperstructureConstants.Pose.shootBarge, 1),
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
        Set.of(myIntake, myIntakeLow, myElevatorWrist));
  }

  public static Command CoralL4Cmd(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.parallel(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.CoralL2_4),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "L4")),
        Commands.either(
            Commands.sequence(
                myElevatorWrist.setPositionCmd(SuperstructureConstants.Pose.L4),
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
                myElevatorWrist.setPositionCmd(SuperstructureConstants.Pose.L3),
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
                myElevatorWrist.setPositionCmd(SuperstructureConstants.Pose.L2),
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
            myElevatorWrist.setPositionCmd(SuperstructureConstants.Pose.L1, 1),
            Commands.none(),
            () -> {
              return indexing || havePiece || RobotContainer.isM_overideMode();
            }));
  }

  public static Command CoralIntakePositionCmd(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.CoralL2_4),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "INTAKE")),
        myElevatorWrist.setPositionCmd(SuperstructureConstants.Pose.intakeCoral));
  }

  public static Command AlgaeToNetCmd(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.AlgaeNet),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "NET")),
        myElevatorWrist.setPositionCmd(SuperstructureConstants.Pose.preBarge, 1));
  }

  public static Command AlgaeToP1(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.Algae),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "P1")),
        myElevatorWrist.setPositionCmd(SuperstructureConstants.Pose.processor));
  }

  public static Command AlgaeAtA2(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.Algae),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "A2")),
        myElevatorWrist.setPositionCmd(SuperstructureConstants.Pose.A2));
  }

  public static Command AlgaeAtA1(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.Algae),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "A1")),
        myElevatorWrist.setPositionCmd(SuperstructureConstants.Pose.A1));
  }

  public static Command AlgaeCradle(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.Algae),
        Commands.runOnce(
            () -> Logger.recordOutput("Manipulator/ElevatorWristState", "AlgaeCradle")),
        myElevatorWrist.setPositionCmd(
            SuperstructureConstants.ELEVATOR.min, SuperstructureConstants.WRIST.safe),
        myElevatorWrist.setPositionCmd(SuperstructureConstants.Pose.cradle));
  }

  public static Command DeployClimberCmd(Climber myClimber, Lights LED) {
    return Commands.sequence(
        Commands.runOnce(() -> LED.setSingleFadeAnimation(LEDColor.RED, LEDSegment.ALL)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "deploying")),
        Commands.runOnce(() -> myClimber.setPosition(Constants.CLIMBER.DEPLOY), myClimber),
        Commands.waitUntil(() -> myClimber.atPosition()),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "deployed")),
        Commands.runOnce(() -> LED.setSolidColor(LEDColor.RED, LEDSegment.ALL)));
  }

  public static Command RetractClimberCmd(Climber myClimber, Lights LED) {
    return Commands.sequence(
        Commands.runOnce(() -> LED.setSingleFadeAnimation(LEDColor.GREEN, LEDSegment.ALL)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "climbing")),
        Commands.runOnce(() -> myClimber.setPosition(Constants.CLIMBER.RETRACT), myClimber),
        Commands.waitUntil(() -> myClimber.atPosition()),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "CLIMBED")),
        Commands.runOnce(
            () -> LED.setRainbowAnimation(LEDSegment.ALL, AnimationDirectionValue.Forward)));
  }

  public static Command climberToZeroCmd(Climber myClimber, Lights LED) {
    return Commands.sequence(
        Commands.runOnce(() -> LED.setSingleFadeAnimation(LEDColor.RED, LEDSegment.ALL)),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "to zero")),
        Commands.runOnce(() -> myClimber.setPosition(0), myClimber),
        Commands.waitUntil(() -> myClimber.atPosition()),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "at zero")),
        Commands.runOnce(
            () -> LED.setRainbowAnimation(LEDSegment.ALL, AnimationDirectionValue.Forward)));
  }

  public static Command ElevatorWristZeroCmd(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.sequence(
        Commands.runOnce(() -> scoringPosition = ScoringPosition.CoralL2_4),
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ElevatorWristState", "ZERO")),
        myElevatorWrist.setPositionCmd(
            SuperstructureConstants.ELEVATOR.min, SuperstructureConstants.WRIST.min));
  }

  public static Command BumpClimberCmd(Climber myClimber, double bumpValue) {
    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput("Manipulator/ClimberState", "bumping")),
        Commands.runOnce(
            () -> myClimber.setPosition(myClimber.getPosition() + bumpValue), myClimber),
        Commands.waitUntil(() -> myClimber.atPosition()),
        Commands.runOnce(() -> myClimber.zero()));
  }

  public static Command FunctionalTest(
      RollerSystem myIntake,
      RollerSystem myIntakeLow,
      ElevatorWristSubsystem myElevatorWrist,
      RangeSensor intake_sensor,
      Climber myClimber,
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

  // TODO fix this
  public static Command TestElevatorWristSequencing(ElevatorWristSubsystem myElevatorWrist) {
    return Commands.runOnce(
        () -> {
          List<Command> positions =
              Arrays.asList(
                  CoralIntakePositionCmd(myElevatorWrist),
                  CoralL2Cmd(myElevatorWrist),
                  CoralL3Cmd(myElevatorWrist),
                  CoralL4Cmd(myElevatorWrist),
                  CoralL1Cmd(myElevatorWrist),
                  AlgaeToP1(myElevatorWrist),
                  AlgaeAtA1(myElevatorWrist),
                  AlgaeAtA2(myElevatorWrist),
                  AlgaeToNetCmd(myElevatorWrist),
                  AlgaeCradle(myElevatorWrist));

          for (int i = 0; i < positions.size(); i++) {
            for (int k = i + 1; k < positions.size(); k++) {
              positions.get(i);
              positions.get(k);
              Commands.waitSeconds(10);
            }
            positions.get(i);
            Commands.waitSeconds(.3);
          }
          ElevatorWristZeroCmd(myElevatorWrist);
        });
  }

  public static Command staticElevatorCharacterization(
      ElevatorWristSubsystem myElevatorWrist, double outputRampRate) {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.sequence(
        myElevatorWrist.setPositionCmd(
            SuperstructureConstants.ELEVATOR.min, SuperstructureConstants.WRIST.safe),
        Commands.waitUntil(() -> myElevatorWrist.atPosition()),
        Commands.waitSeconds(2),
        Commands.runOnce(() -> timer.restart()),
        Commands.run(
                () -> {
                  state.characterizationOutput = outputRampRate * timer.get();
                  myElevatorWrist.getElevator().setVolts(state.characterizationOutput);
                  Logger.recordOutput(
                      "Elevator/StaticCharacterizationOutput", state.characterizationOutput);
                })
            .until(
                () ->
                    myElevatorWrist.getElevator().getPositionRotations()
                        >= SuperstructureConstants.ELEVATOR.max - 0.5)
            .finallyDo(
                () -> {
                  timer.stop();
                  myElevatorWrist
                      .getElevator()
                      .setPosition(myElevatorWrist.getElevator().getPositionRotations());
                  Logger.recordOutput(
                      "Elevator/CharacterizationOutput", state.characterizationOutput);
                }));
  }

  public static Command staticWristCharacterization(
      ElevatorWristSubsystem myElevatorWrist, double outputRampRate) {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.sequence(
        myElevatorWrist.setPositionCmd(
            (SuperstructureConstants.ELEVATOR.maxMidSafe
                    + SuperstructureConstants.ELEVATOR.minMidSafe)
                / 2,
            SuperstructureConstants.WRIST.elevate),
        Commands.waitUntil(() -> myElevatorWrist.atPosition()),
        Commands.waitSeconds(2),
        Commands.runOnce(() -> timer.restart()),
        Commands.run(
                () -> {
                  state.characterizationOutput = outputRampRate * timer.get();
                  myElevatorWrist.getWrist().setVolts(state.characterizationOutput);
                  Logger.recordOutput(
                      "Wrist/StaticCharacterizationOutput", state.characterizationOutput);
                },
                myElevatorWrist)
            .until(
                () ->
                    myElevatorWrist.getWrist().getPosition()
                        >= SuperstructureConstants.WRIST.max - 0.05)
            .finallyDo(
                () -> {
                  timer.stop();
                  myElevatorWrist.getWrist().setPosition(myElevatorWrist.getWrist().getPosition());
                  Logger.recordOutput("Wrist/CharacterizationOutput", state.characterizationOutput);
                }));
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }
}
