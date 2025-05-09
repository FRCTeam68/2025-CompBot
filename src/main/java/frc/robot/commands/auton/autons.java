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

package frc.robot.commands.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.subsystems.ElevatorWristSubsystem;
import frc.robot.subsystems.RangeSensorSubsystem;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.rollers.RollerSystem;
import java.util.List;

public class autons {

  /**
   * Get a list of every path in the given auto
   *
   * @param pathGroupName The name of the path group to load
   * @return Array of paths
   */
  private static PathPlannerPath[] pathBuilder(String pathGroupName) {
    try {
      List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(pathGroupName);

      PathPlannerPath[] pathIndividual = new PathPlannerPath[pathGroup.size()];

      for (int i = 0; i < pathGroup.size(); i++) {
        pathIndividual[i] = pathGroup.get(i);
      }

      return pathIndividual;
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return new PathPlannerPath[1];
    }
  }

  /**
   * Builds a command to follow a path
   *
   * @param pathName The name of the path to load
   * @return A path following command with for the given path
   */
  private static Command followPath(String pathName) {
    try {
      PathPlannerPath pathIndividual = PathPlannerPath.fromPathFile(pathName);

      return followPath(pathIndividual);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());

      return Commands.none();
    }
  }

  /**
   * Builds a command to follow a path
   *
   * @param pathName The path to follow
   * @return A path following command with for the given path
   */
  private static Command followPath(PathPlannerPath path) {
    try {
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());

      return Commands.none();
    }
  }

  public static Command side(
      Boolean leftSide,
      RollerSystem myIntake,
      RollerSystem myIntakeLow,
      ElevatorWristSubsystem myElevatorWrist,
      RangeSensorSubsystem intake_sensor,
      Lights LED) {

    PathPlannerPath[] path = pathBuilder(leftSide ? "AUTON_LEFT1" : "AUTON_RIGHT1");

    return Commands.sequence(
        // first coral
        Commands.parallel(
            followPath(path[0]), // to reef post
            Commands.sequence(
                Commands.runOnce(() -> ManipulatorCommands.setHavePiece(true)),
                Commands.waitSeconds(Constants.AUTO.START_ELEVATOR_DELAY),
                ManipulatorCommands.CoralL4Cmd(myElevatorWrist))),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED),
        // second coral
        Commands.deadline(
            Commands.sequence(
                followPath(path[1]), // to coral station
                Commands.waitSeconds(Constants.AUTO.CORAL_STATION_WAIT),
                followPath(path[2]), // to reef coral 2
                Commands.waitUntil(() -> ManipulatorCommands.isHavePiece())
                    .withTimeout(Constants.AUTO.REEF_TIMEOUT),
                Commands.waitSeconds(Constants.AUTO.INDEX_DELAY)
                    .onlyWhile(() -> ManipulatorCommands.isIndexing())),
            ManipulatorCommands.intakeCmd(
                    myIntake, myIntakeLow, myElevatorWrist, intake_sensor, LED)
                .handleInterrupt(() -> myIntake.setSpeedCmd(0))),
        Commands.either(
            Commands.sequence(
                ManipulatorCommands.CoralL4Cmd(myElevatorWrist),
                Commands.waitUntil(() -> ElevatorWristSubsystem.reefPostDetected)
                    .withTimeout(Constants.AUTO.REEF_POST_TIMEOUT),
                Commands.either(
                    ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED),
                    Commands.none(),
                    () -> {
                      return ElevatorWristSubsystem.reefPostDetected;
                    })),
            // second coral - second try
            Commands.sequence(
                Commands.deadline(
                    Commands.sequence(
                        followPath(path[3]), // to coral station
                        Commands.waitSeconds(Constants.AUTO.CORAL_STATION_WAIT),
                        followPath(path[4]), // to reef coral 2
                        Commands.waitUntil(() -> ManipulatorCommands.isHavePiece())
                            .withTimeout(Constants.AUTO.REEF_TIMEOUT),
                        Commands.waitSeconds(Constants.AUTO.INDEX_DELAY)
                            .onlyWhile(() -> ManipulatorCommands.isIndexing())),
                    ManipulatorCommands.intakeCmd(
                            myIntake, myIntakeLow, myElevatorWrist, intake_sensor, LED)
                        .handleInterrupt(() -> myIntake.setSpeedCmd(0))),
                Commands.either(
                    Commands.sequence(
                        ManipulatorCommands.CoralL4Cmd(myElevatorWrist),
                        Commands.waitUntil(() -> ElevatorWristSubsystem.reefPostDetected)
                            .withTimeout(Constants.AUTO.REEF_POST_TIMEOUT),
                        Commands.either(
                            ManipulatorCommands.shootCmd(
                                myIntake, myIntakeLow, myElevatorWrist, LED),
                            Commands.none(),
                            () -> {
                              return ElevatorWristSubsystem.reefPostDetected;
                            })),
                    Commands.none(),
                    () -> {
                      return ManipulatorCommands.isHavePiece();
                    })),
            () -> {
              return ManipulatorCommands.isHavePiece();
            }),
        // third coral
        Commands.deadline(
            Commands.sequence(
                followPath(path[3]), // to coral station
                Commands.waitSeconds(Constants.AUTO.CORAL_STATION_WAIT),
                followPath(path[4]), // to reef coral 2
                Commands.waitUntil(() -> ManipulatorCommands.isHavePiece())
                    .withTimeout(Constants.AUTO.REEF_TIMEOUT),
                Commands.waitSeconds(Constants.AUTO.INDEX_DELAY)
                    .onlyWhile(() -> ManipulatorCommands.isIndexing())),
            ManipulatorCommands.intakeCmd(
                    myIntake, myIntakeLow, myElevatorWrist, intake_sensor, LED)
                .handleInterrupt(() -> myIntake.setSpeedCmd(0))),
        Commands.either(
            Commands.sequence(
                ManipulatorCommands.CoralL4Cmd(myElevatorWrist),
                Commands.waitUntil(() -> ElevatorWristSubsystem.reefPostDetected)
                    .withTimeout(Constants.AUTO.REEF_POST_TIMEOUT),
                Commands.either(
                    ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED),
                    Commands.none(),
                    () -> {
                      return ElevatorWristSubsystem.reefPostDetected;
                    })),
            // third coral - second try
            Commands.sequence(
                Commands.deadline(
                    Commands.sequence(
                        followPath(path[5]), // to coral station
                        Commands.waitSeconds(Constants.AUTO.CORAL_STATION_WAIT),
                        followPath(path[4]), // to reef coral 2
                        Commands.waitUntil(() -> ManipulatorCommands.isHavePiece())
                            .withTimeout(Constants.AUTO.REEF_TIMEOUT),
                        Commands.waitSeconds(Constants.AUTO.INDEX_DELAY)
                            .onlyWhile(() -> ManipulatorCommands.isIndexing())),
                    ManipulatorCommands.intakeCmd(
                            myIntake, myIntakeLow, myElevatorWrist, intake_sensor, LED)
                        .handleInterrupt(() -> myIntake.setSpeedCmd(0))),
                Commands.either(
                    Commands.sequence(
                        ManipulatorCommands.CoralL4Cmd(myElevatorWrist),
                        Commands.waitUntil(() -> ElevatorWristSubsystem.reefPostDetected)
                            .withTimeout(Constants.AUTO.REEF_POST_TIMEOUT),
                        Commands.either(
                            ManipulatorCommands.shootCmd(
                                myIntake, myIntakeLow, myElevatorWrist, LED),
                            Commands.none(),
                            () -> {
                              return ElevatorWristSubsystem.reefPostDetected;
                            })),
                    Commands.none(),
                    () -> {
                      return ManipulatorCommands.isHavePiece();
                    })),
            () -> {
              return ManipulatorCommands.isHavePiece();
            }),
        Commands.parallel(
            ManipulatorCommands.intakeCmd(
                myIntake, myIntakeLow, myElevatorWrist, intake_sensor, LED),
            followPath(path[5]) // to coral station
            ));
  }

  public static Command centerProcessor(
      RollerSystem myIntake,
      RollerSystem myIntakeLow,
      ElevatorWristSubsystem myElevatorWrist,
      RangeSensorSubsystem intake_sensor,
      Lights LED) {

    PathPlannerPath[] path = pathBuilder("AUTON_CENTER_PROCESSOR");

    return Commands.sequence(
        Commands.runOnce(() -> ManipulatorCommands.setHavePiece(true)),
        Commands.parallel(
            followPath(path[0]), // to reef post
            ManipulatorCommands.CoralL4Cmd(myElevatorWrist)),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED),
        Commands.parallel(
            Commands.sequence(
                followPath(path[1]), // away from reef
                Commands.waitSeconds(Constants.AUTO.H4_GH_PATH_DELAY),
                followPath(path[2]) // to reef algae
                ),
            Commands.sequence(
                ManipulatorCommands.AlgaeAtA1(myElevatorWrist),
                ManipulatorCommands.intakeCmd(
                    myIntake, myIntakeLow, myElevatorWrist, intake_sensor, LED))),
        Commands.parallel(
            followPath(path[3]), // to processor
            Commands.sequence(
                Commands.waitSeconds(Constants.AUTO.GH_PROC_ELEVATOR_DELAY),
                ManipulatorCommands.AlgaeToP1(myElevatorWrist))),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED),
        Commands.parallel(
            followPath(path[4]), // to reef algae 2
            Commands.sequence(
                Commands.waitSeconds(Constants.AUTO.PROC_EF_ELEVATOR_DELAY),
                ManipulatorCommands.AlgaeAtA2(myElevatorWrist),
                ManipulatorCommands.intakeCmd(
                    myIntake, myIntakeLow, myElevatorWrist, intake_sensor, LED))),
        Commands.parallel(
            followPath(path[5]), // to processor
            Commands.sequence(
                Commands.waitSeconds(Constants.AUTO.EF_PROC_ELEVATOR_DELAY),
                ManipulatorCommands.AlgaeToP1(myElevatorWrist))),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED));
  }

  public static Command centerNet(
      RollerSystem myIntake,
      RollerSystem myIntakeLow,
      ElevatorWristSubsystem myElevatorWrist,
      RangeSensorSubsystem intake_sensor,
      Lights LED,
      Boolean playoff) {
    // if (playoff) {
    //   PathPlannerPath[] path = pathBuilder("PLAYOFFS_AUTON_CENTER_NET");
    // } else {
    //   PathPlannerPath[] path = pathBuilder("AUTON_CENTER_NET");
    // }
    PathPlannerPath[] path =
        pathBuilder(playoff ? "PLAYOFFS_AUTON_CENTER_NET" : "AUTON_CENTER_NET");
    return Commands.sequence(
        Commands.runOnce(() -> ManipulatorCommands.setHavePiece(true)),
        Commands.parallel(
            followPath(path[0]), // to reef post
            ManipulatorCommands.CoralL4Cmd(myElevatorWrist)),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED),
        Commands.parallel(
            Commands.sequence(
                followPath(path[1]), // away from reef
                Commands.waitSeconds(Constants.AUTO.H4_GH_PATH_DELAY),
                followPath(path[2])), // to reef algae
            Commands.sequence(
                ManipulatorCommands.AlgaeAtA1(myElevatorWrist),
                ManipulatorCommands.intakeCmd(
                    myIntake, myIntakeLow, myElevatorWrist, intake_sensor, LED))),
        Commands.parallel(
            followPath(path[3]), // to net
            Commands.sequence(
                Commands.waitSeconds(Constants.AUTO.GH_NET_ELEVATOR_DELAY),
                ManipulatorCommands.AlgaeToNetCmd(myElevatorWrist))),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED),
        Commands.parallel(
            followPath(path[4]), // to away from reef algae 2
            ManipulatorCommands.AlgaeAtA2(myElevatorWrist)),
        Commands.parallel(
            followPath(path[5]), // to reef algae 2
            ManipulatorCommands.intakeCmd(
                myIntake, myIntakeLow, myElevatorWrist, intake_sensor, LED)),
        Commands.parallel(
            followPath(path[6]), // to net
            Commands.sequence(
                Commands.waitSeconds(Constants.AUTO.IJ_NET_ELEVATOR_DELAY),
                ManipulatorCommands.AlgaeToNetCmd(myElevatorWrist))),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED),
        Commands.parallel(
            followPath(path[7]),
            ManipulatorCommands.CoralIntakePositionCmd(
                myElevatorWrist))); // to away from reef algae 3
    //     ManipulatorCommands.AlgaeAtA2(myElevatorWrist)),
    // Commands.parallel(
    //     followPath(path[8]), // to reef algae 3
    //     ManipulatorCommands.intakeCmd(myIntake, myIntakeLow, myElevatorWrist, intake_sensor)),
    // Commands.parallel(
    //     followPath(path[9]), // to net
    //     Commands.sequence(
    //         Commands.waitSeconds(Constants.AUTO.EF_NET_ELEVATOR_DELAY),
    //         ManipulatorCommands.AlgaeToNetCmd(myElevatorWrist))),
    // ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist),
    // ManipulatorCommands.CoralIntakePositionCmd(myElevatorWrist));

  }
}
