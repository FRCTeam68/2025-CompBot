package frc.robot.commands.auton.autonSequences;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.commands.auton.AutonPathCommands;
import frc.robot.subsystems.ElevatorWristSubsystem;
import frc.robot.subsystems.RangeSensorSubsystem;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.rollers.RollerSystem;

public class AutonSide {
  public static Command sequence(
      RollerSystem myIntake,
      RollerSystem myIntakeLow,
      ElevatorWristSubsystem myElevatorWrist,
      RangeSensorSubsystem intake_sensor,
      Lights LED) {

    return Commands.sequence(
        // first coral
        Commands.parallel(
            AutonPathCommands.followPath(0), // to reef post
            Commands.sequence(
                Commands.runOnce(() -> ManipulatorCommands.setHavePiece(true)),
                Commands.waitSeconds(Constants.AUTO.START_ELEVATOR_DELAY),
                ManipulatorCommands.CoralL4Cmd(myElevatorWrist))),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED),
        // second coral
        Commands.deadline(
            Commands.sequence(
                AutonPathCommands.followPath(1), // to coral station
                Commands.waitSeconds(Constants.AUTO.CORAL_STATION_WAIT),
                AutonPathCommands.followPath(2), // to reef coral 2
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
                        AutonPathCommands.followPath(3), // to coral station
                        Commands.waitSeconds(Constants.AUTO.CORAL_STATION_WAIT),
                        AutonPathCommands.followPath(4), // to reef coral 2
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
                AutonPathCommands.followPath(3), // to coral station
                Commands.waitSeconds(Constants.AUTO.CORAL_STATION_WAIT),
                AutonPathCommands.followPath(4), // to reef coral 2
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
                        AutonPathCommands.followPath(5), // to coral station
                        Commands.waitSeconds(Constants.AUTO.CORAL_STATION_WAIT),
                        AutonPathCommands.followPath(4), // to reef coral 2
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
            AutonPathCommands.followPath(4) // to coral station
            ));
  }
}
