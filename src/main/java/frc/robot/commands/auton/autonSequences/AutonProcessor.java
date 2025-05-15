package frc.robot.commands.auton.autonSequences;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.commands.auton.AutonPathCommands;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.sensors2.CoralSensor;
import frc.robot.subsystems.superstructure.ElevatorWristSubsystem;

public class AutonProcessor {
  public static Command sequence(
      RollerSystem myIntake,
      RollerSystem myIntakeLow,
      ElevatorWristSubsystem myElevatorWrist,
      CoralSensor intake_sensor,
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
                AutonPathCommands.followPath(2) // to reef algae
                ),
            Commands.sequence(
                ManipulatorCommands.AlgaeAtA1(myElevatorWrist),
                ManipulatorCommands.intakeCmd(
                    myIntake, myIntakeLow, myElevatorWrist, intake_sensor, LED))),
        Commands.parallel(
            AutonPathCommands.followPath(3), // to processor
            Commands.sequence(
                Commands.waitSeconds(Constants.AUTO.GH_PROC_ELEVATOR_DELAY),
                ManipulatorCommands.AlgaeToP1(myElevatorWrist))),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED),
        Commands.parallel(
            AutonPathCommands.followPath(4), // to reef algae 2
            Commands.sequence(
                Commands.waitSeconds(Constants.AUTO.PROC_EF_ELEVATOR_DELAY),
                ManipulatorCommands.AlgaeAtA2(myElevatorWrist),
                ManipulatorCommands.intakeCmd(
                    myIntake, myIntakeLow, myElevatorWrist, intake_sensor, LED))),
        Commands.parallel(
            AutonPathCommands.followPath(5), // to processor
            Commands.sequence(
                Commands.waitSeconds(Constants.AUTO.EF_PROC_ELEVATOR_DELAY),
                ManipulatorCommands.AlgaeToP1(myElevatorWrist))),
        ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED));
  }
}
