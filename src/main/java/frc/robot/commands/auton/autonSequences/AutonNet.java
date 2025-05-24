package frc.robot.commands.auton.autonSequences;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.commands.auton.AutonPathCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.sensors.RangeSensor;
import frc.robot.subsystems.superstructure.ElevatorWristSubsystem;

public class AutonNet extends Command {
  private RollerSystem myIntake;
  private RollerSystem myIntakeLow;
  private ElevatorWristSubsystem myElevatorWrist;
  private RangeSensor myCoralIntakeSensor;
  private Lights LED;

  public AutonNet(
      Drive myDrive,
      RollerSystem myIntake,
      RollerSystem myIntakeLow,
      ElevatorWristSubsystem myElevatorWrist,
      RangeSensor intake_sensor,
      Lights LED) {
    this.myIntake = myIntake;
    this.myIntakeLow = myIntakeLow;
    this.myElevatorWrist = myElevatorWrist;
    this.myCoralIntakeSensor = intake_sensor;
    this.LED = LED;
    addRequirements(myDrive);
  }

  @Override
  public void execute() {
    AutonPathCommands.followPath(0); // to reef post
    ManipulatorCommands.CoralL4Cmd(myElevatorWrist);
    // Commands.sequence(
    //     Commands.runOnce(() -> ManipulatorCommands.setHavePiece(true)),
    //     Commands.parallel(
    //         AutonPathCommands.followPath(0), // to reef post
    //         ManipulatorCommands.CoralL4Cmd(myElevatorWrist)),
    //     ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED),
    //     Commands.parallel(
    //         Commands.sequence(
    //             AutonPathCommands.followPath(1), // away from reef
    //             Commands.waitSeconds(Constants.AUTO.H4_GH_PATH_DELAY),
    //             AutonPathCommands.followPath(2)), // to reef algae
    //         Commands.sequence(
    //             ManipulatorCommands.AlgaeAtA1(myElevatorWrist),
    //             ManipulatorCommands.intakeCmd(
    //                 myIntake, myIntakeLow, myElevatorWrist, myCoralIntakeSensor, LED))),
    //     Commands.parallel(
    //         AutonPathCommands.followPath(3), // to net
    //         Commands.sequence(
    //             Commands.waitSeconds(Constants.AUTO.GH_NET_ELEVATOR_DELAY),
    //             ManipulatorCommands.AlgaeToNetCmd(myElevatorWrist))),
    //     ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED),
    //     Commands.parallel(
    //         AutonPathCommands.followPath(4), // to away from reef algae 2
    //         ManipulatorCommands.AlgaeAtA2(myElevatorWrist)),
    //     Commands.parallel(
    //         AutonPathCommands.followPath(5), // to reef algae 2
    //         ManipulatorCommands.intakeCmd(
    //             myIntake, myIntakeLow, myElevatorWrist, myCoralIntakeSensor, LED)),
    //     Commands.parallel(
    //         AutonPathCommands.followPath(6), // to net
    //         Commands.sequence(
    //             Commands.waitSeconds(Constants.AUTO.IJ_NET_ELEVATOR_DELAY),
    //             ManipulatorCommands.AlgaeToNetCmd(myElevatorWrist))),
    //     ManipulatorCommands.shootCmd(myIntake, myIntakeLow, myElevatorWrist, LED),
    //     Commands.parallel(
    //         AutonPathCommands.followPath(7),
    //         ManipulatorCommands.CoralIntakePositionCmd(
    //             myElevatorWrist))); // to away from reef algae 3
    //
    //
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
