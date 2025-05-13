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

public class AutonNet {
  public static Command sequence(
      RollerSystem myIntake,
      RollerSystem myIntakeLow,
      ElevatorWristSubsystem myElevatorWrist,
      RangeSensorSubsystem intake_sensor,
      Lights LED) {

    return Commands.sequence(
        Commands.runOnce(() -> ManipulatorCommands.setHavePiece(true)),
        Commands.parallel(
            AutonPathCommands.followPath(0), // to reef post
            ManipulatorCommands.CoralL4Cmd(myElevatorWrist)),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED),
        Commands.parallel(
            Commands.sequence(
                AutonPathCommands.followPath(1), // away from reef
                Commands.waitSeconds(Constants.AUTO.H4_GH_PATH_DELAY),
                AutonPathCommands.followPath(2)), // to reef algae
            Commands.sequence(
                ManipulatorCommands.AlgaeAtA1(myElevatorWrist),
                ManipulatorCommands.intakeCmd(
                    myIntake, myIntakeLow, myElevatorWrist, intake_sensor, LED))),
        Commands.parallel(
            AutonPathCommands.followPath(3), // to net
            Commands.sequence(
                Commands.waitSeconds(Constants.AUTO.GH_NET_ELEVATOR_DELAY),
                ManipulatorCommands.AlgaeToNetCmd(myElevatorWrist))),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED),
        Commands.parallel(
            AutonPathCommands.followPath(4), // to away from reef algae 2
            ManipulatorCommands.AlgaeAtA2(myElevatorWrist)),
        Commands.parallel(
            AutonPathCommands.followPath(5), // to reef algae 2
            ManipulatorCommands.intakeCmd(
                myIntake, myIntakeLow, myElevatorWrist, intake_sensor, LED)),
        Commands.parallel(
            AutonPathCommands.followPath(6), // to net
            Commands.sequence(
                Commands.waitSeconds(Constants.AUTO.IJ_NET_ELEVATOR_DELAY),
                ManipulatorCommands.AlgaeToNetCmd(myElevatorWrist))),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED),
        Commands.parallel(
            AutonPathCommands.followPath(7),
            ManipulatorCommands.CoralIntakePositionCmd(
                myElevatorWrist))); // to away from reef algae 3
    //     ManipulatorCommands.AlgaeAtA2(myElevatorWrist)),
    // Commands.parallel(
    //     AutonPathCommands.followPath(8), // to reef algae 3
    //     ManipulatorCommands.intakeCmd(myIntake, myIntakeLow, myElevatorWrist, intake_sensor)),
    // Commands.parallel(
    //     AutonPathCommands.followPath(9), // to net
    //     Commands.sequence(
    //         Commands.waitSeconds(Constants.AUTO.EF_NET_ELEVATOR_DELAY),
    //         ManipulatorCommands.AlgaeToNetCmd(myElevatorWrist))),
    // ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist),
    // ManipulatorCommands.CoralIntakePositionCmd(myElevatorWrist));
  }
}
