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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants;
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

  public LOCATIONS location;
  public double shootSpeed;

  private RollerSystem myIntakeShooter;
  private RangeSensorSubSystem myIntakeSensor;
  private ElevatorWristSubSystem myElevatorWrist;

  public ManipulatorCommands(
      ElevatorWristSubSystem elevator_wrist,
      RollerSystem intake_shooter,
      RangeSensorSubSystem intake_sensor) {

    myElevatorWrist = elevator_wrist;
    myIntakeShooter = intake_shooter;
    myIntakeSensor = intake_sensor;

    location = LOCATIONS.INTAKE;
    shootSpeed = Constants.INTAKE_SHOOTER.CORAL_INTAKE_SPEED;
  }

  public Command intakeCmd() {
    return Commands.sequence(
        new ConditionalCommand(
            // already at intake, so just spin it up and wait for it
            intakeCoralCmd(),
            new ConditionalCommand(
                // we are at known location of algae for spin up and wait for algae
                intakeAlgaeCmd(),

                // we are at unknown location.  try to return to intake
                // could reach this state if elevator/wrist has to be stopped manually.
                returnToIntakeFromUnknownCmd(),

                // second check if we are at algae intake position, if so intake algae
                () -> {
                  return location == LOCATIONS.A1
                      || location == LOCATIONS.A2
                      || location == LOCATIONS.P1;
                }),
            () -> {
              // first check if we are at intake already, if so just intake coral
              return location == LOCATIONS.INTAKE;
            }));
  }

  public Command intakeCoralCmd() {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.blue, 4)),
        myIntakeShooter.setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_SPEED),
        Commands.waitUntil(() -> myIntakeSensor.havePiece()),
        Commands.waitSeconds(.1),
        myIntakeShooter.setSpeedCmd(0),

        // index to have it flush to front
        myIntakeShooter.setSpeedCmd(Constants.INTAKE_SHOOTER.COREL_INTAKE_INDEX_SPEED),
        Commands.waitUntil(() -> !myIntakeSensor.havePiece()),
        myIntakeShooter.setSpeedCmd(0),
        Commands.runOnce(
            () ->
                myIntakeShooter.setPosition(
                    myIntakeShooter.getPosition()
                        - Constants.INTAKE_SHOOTER.COREL_INTAKE_INDEX_REVERSE)),
        Commands.runOnce(() -> location = LOCATIONS.INTAKE),
        Commands.runOnce(() -> shootSpeed = 0),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.blue)));
  }

  private Command intakeAlgaeCmd() {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.blue, 4)),
        myIntakeShooter.setSpeedCmd(Constants.INTAKE_SHOOTER.ALGAE_INTAKE_SPEED),
        Commands.waitUntil(() -> myIntakeShooter.hasPiece()),
        Commands.runOnce(() -> myIntakeShooter.stop()),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.blue)));
  }

  public Command shootCmd(boolean do_not_start_intake) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.red)),
        new ConditionalCommand(
            ShootAlgaeToNetCmd(),
            Commands.none(),
            () -> {
              return location == LOCATIONS.PRENET;
            }),

        // shoot according to saved shooter speed
        myIntakeShooter.setSpeedCmd(shootSpeed),
        Commands.waitUntil(() -> myIntakeSensor.havePiece() == false),
        Commands.waitSeconds(1),
        myIntakeShooter.setSpeedCmd(0),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)),
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        new ConditionalCommand(
            myElevatorWrist.setPositionWristCmd(Constants.WRIST.CRADLE, true),
            Commands.none(),
            () -> {
              // if location is any of these, then need to move wrist to intermediate position first
              return location == LOCATIONS.P1
                  || location == LOCATIONS.A1
                  || location == LOCATIONS.A2
                  || location == LOCATIONS.L4PRIME;
            }),
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.INTAKE, true),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.INTAKE, true),
        Commands.runOnce(() -> SmartDashboard.putString("atPosition", "INTAKE")),
        Commands.runOnce(() -> location = LOCATIONS.INTAKE),
        Commands.runOnce(() -> shootSpeed = 0),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)),
        intakeCoralCmd().unless(() -> do_not_start_intake));
  }

  private Command ShootAlgaeToNetCmd() {
    return Commands.sequence(
        // Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.red, 4)),
        myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.SHOOTNET, false),
        Commands.waitSeconds(.5),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.SHOOTNET, false),
        Commands.waitSeconds(.1),
        myIntakeShooter.setSpeedCmd(60),
        Commands.waitSeconds(2),
        myIntakeShooter.setSpeedCmd(0)
        // Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange))
        );
  }

  public Command IntakeToL4primeCmd() {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> location = LOCATIONS.TRANSITIONING),
        Commands.runOnce(() -> shootSpeed = 0),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.SHOOTNET, true),
        myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.L4PRIME, true),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.L4PRIME, true),
        Commands.runOnce(() -> SmartDashboard.putString("atPosition", "L4PRIME")),
        Commands.runOnce(() -> location = LOCATIONS.L4PRIME),
        Commands.runOnce(() -> shootSpeed = Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public Command IntakeToL4Cmd() {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> location = LOCATIONS.TRANSITIONING),
        Commands.runOnce(() -> shootSpeed = 0),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.L4, true),
        myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.L4, true),
        Commands.runOnce(() -> SmartDashboard.putString("atPosition", "L4")),
        Commands.runOnce(() -> location = LOCATIONS.L4),
        Commands.runOnce(() -> shootSpeed = Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public Command IntakeToL3Cmd() {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> location = LOCATIONS.TRANSITIONING),
        Commands.runOnce(() -> shootSpeed = 0),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.L3, true),
        myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.L3, true),
        Commands.runOnce(() -> SmartDashboard.putString("atPosition", "L3")),
        Commands.runOnce(() -> location = LOCATIONS.L3),
        Commands.runOnce(() -> shootSpeed = Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public Command IntakeToL2Cmd() {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> location = LOCATIONS.TRANSITIONING),
        Commands.runOnce(() -> shootSpeed = 0),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.L2, true),
        myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.L2, true),
        Commands.runOnce(() -> SmartDashboard.putString("atPosition", "L2")),
        Commands.runOnce(() -> location = LOCATIONS.L2),
        Commands.runOnce(() -> shootSpeed = Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public Command IntakeToL1Cmd() {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> location = LOCATIONS.TRANSITIONING),
        Commands.runOnce(() -> shootSpeed = 0),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.L1, true),
        myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.L1, true),
        Commands.runOnce(() -> SmartDashboard.putString("atPosition", "L1")),
        Commands.runOnce(() -> location = LOCATIONS.L1),
        Commands.runOnce(() -> shootSpeed = Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public Command IntakeToA2() {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> location = LOCATIONS.TRANSITIONING),
        Commands.runOnce(() -> shootSpeed = 0),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.CRADLE, true),
        myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.A2, true),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.A2, true),
        Commands.runOnce(() -> SmartDashboard.putString("atPosition", "A2")),
        Commands.runOnce(() -> location = LOCATIONS.A2),
        Commands.runOnce(() -> shootSpeed = Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public Command IntakeToA1() {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> location = LOCATIONS.TRANSITIONING),
        Commands.runOnce(() -> shootSpeed = 0),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.CRADLE, true),
        myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.A1, true),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.A1, true),
        Commands.runOnce(() -> SmartDashboard.putString("atPosition", "A1")),
        Commands.runOnce(() -> location = LOCATIONS.A1),
        Commands.runOnce(() -> shootSpeed = Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public Command GoToP1Cmd() {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> location = LOCATIONS.TRANSITIONING),
        Commands.runOnce(() -> shootSpeed = 0),
        new ConditionalCommand(
            myElevatorWrist.setPositionWristCmd(Constants.WRIST.CRADLE, true),
            Commands.none(),
            () -> {
              return location == LOCATIONS.INTAKE;
            }),
        myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.P1, true),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.P1, true),
        Commands.runOnce(() -> SmartDashboard.putString("atPosition", "P1")),
        Commands.runOnce(() -> location = LOCATIONS.P1),
        Commands.runOnce(() -> shootSpeed = Constants.INTAKE_SHOOTER.ALGAE_SHOOT_SPEED),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public Command GoToPreNetCmd() {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
        Commands.runOnce(() -> location = LOCATIONS.TRANSITIONING),
        Commands.runOnce(() -> shootSpeed = 0),
        myElevatorWrist.setPositionWristCmd(Constants.WRIST.PRENET, true),
        myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.PRENET, true),
        Commands.runOnce(() -> SmartDashboard.putString("atPosition", "PRENET")),
        Commands.runOnce(() -> location = LOCATIONS.PRENET),
        Commands.runOnce(() -> shootSpeed = Constants.INTAKE_SHOOTER.ALGAE_SHOOT_SPEED),
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange)));
  }

  public Command returnToIntakeFromUnknownCmd() {
    return Commands.sequence(
        Commands.runOnce(() -> location = LOCATIONS.TRANSITIONING),
        Commands.runOnce(() -> shootSpeed = 0),
        new ConditionalCommand(
            // wrist then elevator
            Commands.sequence(
                Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
                myElevatorWrist.setPositionWristCmd(Constants.WRIST.INTAKE, true),
                myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.INTAKE, true),
                Commands.runOnce(() -> SmartDashboard.putString("atPosition", "INTAKE")),
                Commands.runOnce(() -> location = LOCATIONS.INTAKE),
                Commands.runOnce(() -> shootSpeed = 0),
                Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange))),

            new ConditionalCommand(
                // wrist to safe position, then elevator, then wrist
                Commands.sequence(
                    Commands.runOnce(
                        () -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4)),
                    myElevatorWrist.setPositionWristCmd(Constants.WRIST.CRADLE, true),
                    myElevatorWrist.setPositionElevatorCmd(Constants.ELEVATOR.INTAKE, true),
                    myElevatorWrist.setPositionWristCmd(Constants.WRIST.INTAKE, true),
                    Commands.runOnce(() -> SmartDashboard.putString("atPosition", "INTAKE")),
                    Commands.runOnce(() -> location = LOCATIONS.INTAKE),
                    Commands.runOnce(() -> shootSpeed = 0),
                    Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange))),

                // should not reach this if in valid block
                Commands.runOnce(() -> System.out.println("\tunrecognized area.  cannot return to Intake")),

                () -> {
                  switch (myElevatorWrist.findMyBlockNumber()) {
                    case 47, 45, 42, 35, 29, 27, 26, 23, 17:
                      //  second, if 3 moves, wrist to safe, then elevator, then wrist
                      return true;
                    default:
                      return false;
                  }
                }),
                
            () -> {
              switch (myElevatorWrist.findMyBlockNumber()) {
                case 46, 41, 34, 31, 28, 25, 22, 19, 16, 15, 14, 13, 10, 9, 8, 7, 4, 3, 2, 1, 0:
                  // first, if simple 2 moves: wrist then elevator
                  return true;
                default:
                  return false;
              }
            }));
  }
}
