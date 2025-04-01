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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.subsystems.ElevatorWristSubSystem;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;

public class autons {

  private static PathPlannerPath[] path;
  private static String loadedAutonName;
  private static String pathGroupName;
  private static boolean intake = false;
  private static int i;
  private static boolean intaked;
  private static boolean shot;
  @AutoLogOutput private static Timer timer1 = new Timer(); // from start of auto
  @AutoLogOutput private static Timer timer2 = new Timer();

  public static void startTimer(Timer timer) {
    timer.reset();
    timer.start();
  }

  public static void stopTimer(Timer timer) {
    timer.stop();
    timer.reset();
  }

  public static void stopAllTimers() {
    timer1.stop();
    timer2.stop();
  }

  // public static void pathBuilder(String autonName) {
  //   if (autonName != loadedAutonName) {
  //     try {
  //       switch (autonName) {
  //         case "LEFT":
  //           pathGroupName = "AUTON_LEFT1";
  //           break;
  //         case "RIGHT":
  //           pathGroupName = "AUTON_RIGHT1";
  //           break;
  //         case "CENTER_PROCESSOR":
  //           pathGroupName = "AUTON_CENTER_PROCESSOR";
  //           break;
  //         case "CENTER_NET":
  //           pathGroupName = "AUTON_CENTER_NET";
  //           break;
  //         default:
  //           pathGroupName = null;
  //           break;
  //       }

  //       List<PathPlannerPath> pathGroup =
  // PathPlannerAuto.getPathGroupFromAutoFile(pathGroupName);
  //       PathPlannerPath[] pathIndividual = new PathPlannerPath[pathGroup.size()];

  //       for (int i = 0; i < pathGroup.size(); i++) {
  //         pathIndividual[i] = pathGroup.get(i);
  //       }

  //       loadedAutonName = autonName;
  //       path = pathIndividual;

  //     } catch (Exception e) {
  //       DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
  //       loadedAutonName = null;
  //       path = new PathPlannerPath[1];
  //     }
  //   }
  // }

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

  private static PathPlannerPath pathBuilderIndividual(String pathName) {
    try {
      // Load the path you want to follow using its name in the GUI
      return PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return null;
    }
  }

  //   public static Command side(boolean leftSide) {
  //     startTimer(timer1);
  //     stopTimer(timer2);

  //     PathPlannerPath thirdCoralToSource = pathBuilderIndividual(leftSide ? "L1_L-Source" :
  // "R1_D-Source");
  //     String pathGroupName;
  //     pathGroupName = leftSide ? "AUTON_LEFT1" : "AUTON_RIGHT1";
  //     path = pathBuilder(leftSide ? "AUTON_LEFT1" : "AUTON_RIGHT1");
  //     ManipulatorCommands.setHavePiece(true);

  //     return Commands.sequence(
  //         // first coral
  //         Commands.parallel(
  //             AutoBuilder.followPath(path[0]), // to reef post
  //             Commands.sequence(
  //                 Commands.waitSeconds(Constants.AUTO.START_ELEVATOR_DELAY),
  //                 ManipulatorCommands.CoralL4Cmd())),
  //         ManipulatorCommands.shootCmd()
  //             .withTimeout(Constants.AUTO.CORAL_SHOOT_TIMEOUT)
  //             .handleInterrupt(() -> RobotContainer.intakeShooter.setSpeedCmd(0)),
  //         // second coral
  //         Commands.deadline(
  //             Commands.sequence(
  //                 AutoBuilder.followPath(path[1]), // to coral station
  //                 // Commands.waitSeconds(Constants.AUTO.CORAL_STATION_WAIT),
  //                 Commands.runOnce(() -> startTimer(timer2)),
  //                 AutoBuilder.followPath(path[2]), // to reef
  //                 Commands.waitUntil(() -> ManipulatorCommands.isHavePiece())
  //                     .withTimeout(Constants.AUTO.REEF_TIMEOUT),
  //                 Commands.waitUntil(() -> ManipulatorCommands.isHavePiece())
  //                     .onlyWhile(() -> ManipulatorCommands.isIndexing())
  //                     .withTimeout(Constants.AUTO.INDEX_DELAY)), // to reef coral 2
  //             Commands.sequence(
  //                 Commands.waitUntil(() -> intake == true),
  //                 Commands.runOnce(() -> intake = false),
  //                 ManipulatorCommands.intakeCmd()
  //                     .handleInterrupt(() -> RobotContainer.intakeShooter.setSpeedCmd(0))),
  //             Commands.sequence(
  //                 ManipulatorCommands.CoralIntakePositionCmd().andThen(() -> intake = true),
  //                 Commands.waitUntil(() -> ManipulatorCommands.isSafeToMove()),
  //                 Commands.waitUntil(
  //                     () -> timer2.get() >= Constants.AUTO.CORAL_STATION_ELEVATOR_DELAY),
  //                 ManipulatorCommands.CoralL4Cmd())),
  //         Commands.runOnce(() -> stopTimer(timer2)),
  //         Commands.sequence(
  //                 Commands.runOnce(() -> Logger.recordOutput("Manipulator/test", "SHOOTING")),
  //                 ManipulatorCommands.CoralL4Cmd(),
  //                 Commands.waitUntil(() -> ElevatorWristSubSystem.reefPostDetected)
  //                     .withTimeout(Constants.AUTO.REEF_POST_TIMEOUT),
  //                 ManipulatorCommands.shootCmd()
  //                     .withTimeout(Constants.AUTO.CORAL_SHOOT_TIMEOUT)
  //                     .handleInterrupt(() -> RobotContainer.intakeShooter.setSpeedCmd(0))
  //                     .onlyIf(() -> ElevatorWristSubSystem.reefPostDetected))
  //             .onlyIf(() -> ManipulatorCommands.isHavePiece()),
  //         // third coral
  //         Commands.deadline(
  //             Commands.sequence(
  //                 AutoBuilder.followPath(path[3]), // to coral station
  //                 // Commands.waitSeconds(Constants.AUTO.CORAL_STATION_WAIT),
  //                 Commands.runOnce(() -> startTimer(timer2)),
  //                 AutoBuilder.followPath(path[4]), // to reef
  //                 Commands.waitUntil(() -> ManipulatorCommands.isHavePiece())
  //                     .withTimeout(Constants.AUTO.REEF_TIMEOUT),
  //                 Commands.waitUntil(() -> ManipulatorCommands.isHavePiece())
  //                     .onlyWhile(() -> ManipulatorCommands.isIndexing())
  //                     .withTimeout(Constants.AUTO.INDEX_DELAY)), // to reef coral 2
  //             Commands.sequence(
  //                 Commands.waitUntil(() -> intake == true),
  //                 Commands.runOnce(() -> intake = false),
  //                 ManipulatorCommands.intakeCmd()
  //                     .handleInterrupt(() -> RobotContainer.intakeShooter.setSpeedCmd(0))),
  //             Commands.sequence(
  //                 ManipulatorCommands.CoralIntakePositionCmd().andThen(() -> intake = true),
  //                 Commands.waitUntil(() -> ManipulatorCommands.isSafeToMove()),
  //                 Commands.waitUntil(
  //                     () -> timer2.get() >= Constants.AUTO.CORAL_STATION_ELEVATOR_DELAY),
  //                 ManipulatorCommands.CoralL4Cmd())),
  //         Commands.runOnce(() -> stopTimer(timer2)),
  //         Commands.sequence(
  //                 ManipulatorCommands.CoralL4Cmd(),
  //                 Commands.waitUntil(() -> ElevatorWristSubSystem.isReefPostDetected())
  //                     .withTimeout(Constants.AUTO.REEF_POST_TIMEOUT),
  //                 ManipulatorCommands.shootCmd()
  //                     .withTimeout(Constants.AUTO.CORAL_SHOOT_TIMEOUT)
  //                     .handleInterrupt(() -> RobotContainer.intakeShooter.setSpeedCmd(0))
  //                     .onlyIf(() -> ElevatorWristSubSystem.isReefPostDetected()),
  //                 Commands.parallel(
  //                     AutoBuilder.followPath(path[5]), // to away from reef
  //                     ManipulatorCommands.AlgaeAtA2()),
  //                 Commands.sequence(
  //                         Commands.parallel(
  //                             AutoBuilder.followPath(path[6]), // to reef algae
  //                             ManipulatorCommands.intakeCmd()),
  //                         Commands.parallel(
  //                             AutoBuilder.followPath(path[7]), // to away from reef
  //                             Commands.sequence(
  //                                 Commands.waitSeconds(0.25),
  // ManipulatorCommands.AlgaeCradle())))
  //                     .onlyIf(() -> timer1.get() <= 13),
  //                 Commands.waitUntil(() -> false))
  //             .onlyIf(() -> ManipulatorCommands.isHavePiece()),
  //         Commands.parallel(
  //             AutoBuilder.followPath(thirdCoralToSource),
  //             Commands.parallel(
  //                 ManipulatorCommands.CoralIntakePositionCmd(),
  //                 ManipulatorCommands.intakeCmd()))
  //     );
  //   }

  public static Command side(boolean leftSide) {
    startTimer(timer1);
    stopTimer(timer2);

    PathPlannerPath thirdCoralToSource =
        pathBuilderIndividual(leftSide ? "L1_L-Source" : "R1_D-Source");
    path = pathBuilder(leftSide ? "AUTON_LEFT2" : "AUTON_RIGHT2");
    ManipulatorCommands.setHavePiece(true);
    i = 0;
    intaked = false;
    shot = false;

    return Commands.sequence(
        // first coral
        Commands.parallel(
            AutoBuilder.followPath(path[0]), // to reef post
            Commands.sequence(
                Commands.waitSeconds(Constants.AUTO.START_ELEVATOR_DELAY),
                ManipulatorCommands.CoralL4Cmd())),
        ManipulatorCommands.shootCmd()
            .withTimeout(Constants.AUTO.CORAL_SHOOT_TIMEOUT)
            .handleInterrupt(() -> RobotContainer.intakeShooter.setSpeedCmd(0)),
        // additional coral
        Commands.repeatingSequence(
                Commands.runOnce(() -> intaked = false),
                Commands.runOnce(() -> shot = false),
                Commands.deadline(
                    Commands.sequence(
                        AutoBuilder.followPath(path[1 + i]), // to coral station
                        // Commands.waitSeconds(Constants.AUTO.CORAL_STATION_WAIT),
                        Commands.runOnce(() -> startTimer(timer2)),
                        AutoBuilder.followPath(path[2 + i]), // to reef
                        Commands.waitUntil(() -> ManipulatorCommands.isHavePiece())
                            .withTimeout(Constants.AUTO.REEF_TIMEOUT),
                        Commands.waitUntil(() -> ManipulatorCommands.isHavePiece())
                            .onlyWhile(() -> ManipulatorCommands.isIndexing())
                            .withTimeout(Constants.AUTO.INDEX_DELAY)),
                    Commands.sequence(
                        Commands.waitUntil(() -> intake == true),
                        ManipulatorCommands.intakeCmd()
                            .handleInterrupt(() -> RobotContainer.intakeShooter.setSpeedCmd(0))),
                    Commands.sequence(
                        ManipulatorCommands.CoralIntakePositionCmd().andThen(() -> intake = true),
                        Commands.waitUntil(() -> ManipulatorCommands.isSafeToMove()),
                        Commands.waitUntil(
                            () -> timer2.get() >= Constants.AUTO.CORAL_STATION_ELEVATOR_DELAY),
                        ManipulatorCommands.CoralL4Cmd())),
                Commands.sequence(
                        Commands.runOnce(() -> intaked = true),
                        ManipulatorCommands.CoralL4Cmd(),
                        Commands.waitUntil(() -> ElevatorWristSubSystem.reefPostDetected)
                            .withTimeout(Constants.AUTO.REEF_POST_TIMEOUT),
                        Commands.parallel(
                                ManipulatorCommands.shootCmd(),
                                Commands.runOnce(() -> shot = false))
                            .withTimeout(Constants.AUTO.CORAL_SHOOT_TIMEOUT)
                            .handleInterrupt(() -> RobotContainer.intakeShooter.setSpeedCmd(0))
                            .onlyIf(() -> ElevatorWristSubSystem.reefPostDetected))
                    .onlyIf(() -> ManipulatorCommands.isHavePiece()),
                Commands.runOnce(() -> stopTimer(timer2)),
                Commands.runOnce(() -> intake = false),
                Commands.runOnce(() -> i = i + 2))
            .onlyWhile(() -> i <= 2),
        Commands.either(
                // get algae from reef
                Commands.sequence(
                    Commands.parallel(
                        AutoBuilder.followPath(path[5]), // to away from reef
                        ManipulatorCommands.AlgaeAtA1()),
                    Commands.sequence(
                            Commands.parallel(
                                AutoBuilder.followPath(path[6]), // to reef algae
                                ManipulatorCommands.intakeCmd()),
                            Commands.parallel(
                                AutoBuilder.followPath(path[7]), // to away from reef
                                Commands.waitSeconds(0.25)
                                    .andThen(ManipulatorCommands.AlgaeCradle())))
                        .onlyIf(() -> timer1.get() <= 14)),
                // get 4th coral
                Commands.parallel(
                    AutoBuilder.followPath(thirdCoralToSource), // to coral station
                    Commands.sequence(
                        ManipulatorCommands.CoralIntakePositionCmd(),
                        ManipulatorCommands.intakeCmd())),
                () -> {
                  return (shot);
                  // do nothing if coral is still in the robot
                })
            .onlyIf(() -> (intaked && !shot)));
  }

  public static Command centerProcessor() {
    startTimer(timer1);
    ManipulatorCommands.setHavePiece(true);
    path = pathBuilder("AUTON_CENTER_PROCESSOR");
    return Commands.sequence(
        Commands.parallel(
            AutoBuilder.followPath(path[0]), // to reef post
            ManipulatorCommands.CoralL4Cmd()),
        ManipulatorCommands.shootCmd()
            .withTimeout(Constants.AUTO.CORAL_SHOOT_TIMEOUT)
            .handleInterrupt(() -> RobotContainer.intakeShooter.setSpeedCmd(0)),
        Commands.parallel(
            Commands.sequence(
                AutoBuilder.followPath(path[1]), // away from reef
                Commands.waitSeconds(Constants.AUTO.H4_GH_PATH_DELAY),
                AutoBuilder.followPath(path[2]) // to reef algae
                ),
            Commands.sequence(ManipulatorCommands.AlgaeAtA1(), ManipulatorCommands.intakeCmd())),
        Commands.parallel(
            AutoBuilder.followPath(path[3]), // to processor
            Commands.sequence(
                Commands.waitSeconds(Constants.AUTO.GH_PROC_ELEVATOR_DELAY),
                ManipulatorCommands.AlgaeToP1())),
        ManipulatorCommands.shootCmd(),
        Commands.parallel(
            AutoBuilder.followPath(path[4]), // to reef algae 2
            Commands.sequence(
                Commands.waitSeconds(Constants.AUTO.PROC_EF_ELEVATOR_DELAY),
                ManipulatorCommands.AlgaeAtA2(),
                ManipulatorCommands.intakeCmd())),
        Commands.parallel(
            AutoBuilder.followPath(path[5]), // to processor
            Commands.sequence(
                Commands.waitSeconds(Constants.AUTO.EF_PROC_ELEVATOR_DELAY),
                ManipulatorCommands.AlgaeToP1())),
        ManipulatorCommands.shootCmd());
  }

  public static Command centerNet() {
    startTimer(timer1);
    ManipulatorCommands.setHavePiece(true);
    path = pathBuilder("AUTON_CENTER_NET");
    return Commands.sequence(
        Commands.parallel(
            AutoBuilder.followPath(path[0]), // to reef post
            ManipulatorCommands.CoralL4Cmd()),
        ManipulatorCommands.shootCmd()
            .withTimeout(Constants.AUTO.CORAL_SHOOT_TIMEOUT)
            .handleInterrupt(() -> RobotContainer.intakeShooter.setSpeedCmd(0)),
        Commands.parallel(
            Commands.sequence(
                AutoBuilder.followPath(path[1]), // away from reef
                Commands.waitSeconds(Constants.AUTO.H4_GH_PATH_DELAY),
                AutoBuilder.followPath(path[2])), // to reef algae
            Commands.sequence(ManipulatorCommands.AlgaeAtA1(), ManipulatorCommands.intakeCmd())),
        Commands.parallel(
            AutoBuilder.followPath(path[3]), // to net
            Commands.sequence(
                Commands.waitSeconds(Constants.AUTO.GH_NET_ELEVATOR_DELAY),
                ManipulatorCommands.AlgaeToNetCmd())),
        ManipulatorCommands.shootCmd(),
        Commands.parallel(
            AutoBuilder.followPath(path[4]), // to away from reef algae 2
            ManipulatorCommands.AlgaeAtA2()),
        Commands.parallel(
            AutoBuilder.followPath(path[4]), // to reef algae 2
            ManipulatorCommands.intakeCmd()),
        Commands.parallel(
            AutoBuilder.followPath(path[4]), // to net
            Commands.sequence(
                Commands.waitSeconds(Constants.AUTO.IJ_NET_ELEVATOR_DELAY),
                ManipulatorCommands.AlgaeToNetCmd())),
        ManipulatorCommands.shootCmd(),
        Commands.parallel(
            AutoBuilder.followPath(path[4]),
            ManipulatorCommands.CoralIntakePositionCmd())); // to away from reef algae 3
    //     ManipulatorCommands.AlgaeAtA2(myIntakeLow, myElevatorWrist, false)),
    // Commands.parallel(
    //     AutoBuilder.followPath(path[8]), // to reef algae 3
    //     ManipulatorCommands.intakeCmd(myIntake, myIntakeLow, myElevatorWrist, intake_sensor)),
    // Commands.parallel(
    //     AutoBuilder.followPath(path[9]), // to net
    //     Commands.sequence(
    //         Commands.waitSeconds(Constants.AUTO.EF_NET_ELEVATOR_DELAY),
    //         ManipulatorCommands.AlgaeToNetCmd(myIntakeLow, myElevatorWrist, false))),
    // ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist),
    // ManipulatorCommands.CoralIntakePositionCmd(myIntakeLow, myElevatorWrist));

  }
}
