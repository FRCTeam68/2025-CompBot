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
import frc.robot.subsystems.ElevatorWristSubSystem;
import frc.robot.subsystems.RangeSensorSubSystem;
import frc.robot.subsystems.rollers.RollerSystem;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class autons {

  public static PathPlannerPath[] pathBuilder(String pathGroupName) {
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

  public static Command side(
      Boolean leftSide,
      RollerSystem myIntake,
      RollerSystem myIntakeLow,
      ElevatorWristSubSystem myElevatorWrist,
      RangeSensorSubSystem intake_sensor) {

    String pathGroupName;
    pathGroupName = leftSide ? "AUTON_LEFT1" : "AUTON_RIGHT1";
    PathPlannerPath[] path = pathBuilder(pathGroupName);

    return Commands.sequence(
        Commands.parallel(
            AutoBuilder.followPath(path[0]), // to reef post
            ManipulatorCommands.CoralL4Cmd(myIntakeLow, myElevatorWrist)),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist),
        Commands.deadline(
            Commands.sequence(
                AutoBuilder.followPath(path[1]), // to coral station
                Commands.waitSeconds(Constants.AUTO.CORAL_STATION_WAIT),
                AutoBuilder.followPath(path[2])), // to reef coral 2
            ManipulatorCommands.intakeCmd(myIntake, myIntakeLow, myElevatorWrist, intake_sensor)),
        ManipulatorCommands.intakeCmd(myIntake, myIntakeLow, myElevatorWrist, intake_sensor)
            .withTimeout(Constants.AUTO.REEF_TIMEOUT)
            .handleInterrupt(() -> myIntake.setSpeedCmd(0)),
        Commands.runOnce(
            () -> Logger.recordOutput("Manipulator/test", ManipulatorCommands.havePiece)),
        Commands.either(
            Commands.sequence(
                ManipulatorCommands.CoralL4Cmd(myIntakeLow, myElevatorWrist),
                ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist)),
            Commands.none(),
            () -> {
              return intake_sensor.havePiece();
            }),
        Commands.deadline(
            Commands.sequence(
                AutoBuilder.followPath(path[3]), // to coral station
                Commands.waitSeconds(Constants.AUTO.CORAL_STATION_WAIT),
                AutoBuilder.followPath(path[4])), // to reef coral 2
            ManipulatorCommands.intakeCmd(myIntake, myIntakeLow, myElevatorWrist, intake_sensor)));
  }

  public static Command centerProcessor(
      RollerSystem myIntake,
      RollerSystem myIntakeLow,
      ElevatorWristSubSystem myElevatorWrist,
      RangeSensorSubSystem intake_sensor) {

    PathPlannerPath[] path = pathBuilder("AUTON_CENTER_PROCESSOR");

    return Commands.sequence(
        Commands.parallel(
            AutoBuilder.followPath(path[0]), // to reef post
            ManipulatorCommands.CoralL4Cmd(myIntakeLow, myElevatorWrist)),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist),
        Commands.parallel(
            Commands.sequence(
                AutoBuilder.followPath(path[1]), // away from reef
                AutoBuilder.followPath(path[2])), // to reef algae
            Commands.sequence(
                ManipulatorCommands.AlgaeAtA1(myIntakeLow, myElevatorWrist, false),
                ManipulatorCommands.intakeCmd(
                    myIntake, myIntakeLow, myElevatorWrist, intake_sensor))),
        Commands.parallel(
            AutoBuilder.followPath(path[3]), // to processor
            Commands.sequence(
                Commands.waitSeconds(0.8),
                ManipulatorCommands.AlgaeToP1(myIntakeLow, myElevatorWrist, false))),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist),
        Commands.parallel(
            AutoBuilder.followPath(path[4]), // to reef algae 2
            Commands.sequence(
                Commands.waitSeconds(0.1),
                ManipulatorCommands.AlgaeAtA2(myIntakeLow, myElevatorWrist, false),
                ManipulatorCommands.intakeCmd(
                    myIntake, myIntakeLow, myElevatorWrist, intake_sensor))),
        Commands.parallel(
            AutoBuilder.followPath(path[5]), // to processor
            Commands.sequence(
                Commands.waitSeconds(0.6),
                ManipulatorCommands.AlgaeToP1(myIntakeLow, myElevatorWrist, false))),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist));
  }

  public static Command centerNet(
      RollerSystem myIntake,
      RollerSystem myIntakeLow,
      ElevatorWristSubSystem myElevatorWrist,
      RangeSensorSubSystem intake_sensor) {

    PathPlannerPath[] path = pathBuilder("AUTON_CENTER_NET");

    return Commands.sequence(
        Commands.parallel(
            AutoBuilder.followPath(path[0]), // to reef post
            ManipulatorCommands.CoralL4Cmd(myIntakeLow, myElevatorWrist)),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist),
        Commands.parallel(
            Commands.sequence(
                AutoBuilder.followPath(path[1]), // away from reef
                AutoBuilder.followPath(path[2])), // to reef algae
            Commands.sequence(
                ManipulatorCommands.AlgaeAtA1(myIntakeLow, myElevatorWrist, false),
                ManipulatorCommands.intakeCmd(
                    myIntake, myIntakeLow, myElevatorWrist, intake_sensor))),
        Commands.parallel(
            AutoBuilder.followPath(path[3]), // to net
            Commands.sequence(
                Commands.waitSeconds(0.3),
                ManipulatorCommands.AlgaeToNetCmd(myIntakeLow, myElevatorWrist, false))),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist),
        Commands.parallel(
            AutoBuilder.followPath(path[4]), // to away from reef algae 2
            ManipulatorCommands.AlgaeAtA2(myIntakeLow, myElevatorWrist, false)),
        Commands.parallel(
            AutoBuilder.followPath(path[5]), // to reef algae 2
            ManipulatorCommands.intakeCmd(myIntake, myIntakeLow, myElevatorWrist, intake_sensor)),
        Commands.parallel(
            AutoBuilder.followPath(path[6]), // to net
            Commands.sequence(
                Commands.waitSeconds(0.8),
                ManipulatorCommands.AlgaeToNetCmd(myIntakeLow, myElevatorWrist, false))),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist),
        Commands.parallel(
            AutoBuilder.followPath(path[7]), // to away from reef algae 3
            ManipulatorCommands.AlgaeAtA2(myIntakeLow, myElevatorWrist, false)),
        Commands.parallel(
            AutoBuilder.followPath(path[8]), // to reef algae 3
            ManipulatorCommands.intakeCmd(myIntake, myIntakeLow, myElevatorWrist, intake_sensor)),
        Commands.parallel(
            AutoBuilder.followPath(path[9]), // to net
            Commands.sequence(
                Commands.waitSeconds(2),
                ManipulatorCommands.AlgaeToNetCmd(myIntakeLow, myElevatorWrist, false))),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist),
        ManipulatorCommands.CoralIntakePositionCmd(myIntakeLow, myElevatorWrist));
  }
}
