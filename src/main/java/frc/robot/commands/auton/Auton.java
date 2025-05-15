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

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants;
import frc.robot.commands.auton.autonSequences.AutonNet;
import frc.robot.commands.auton.autonSequences.AutonProcessor;
import frc.robot.commands.auton.autonSequences.AutonSide;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.sensors.CoralSensor;
import frc.robot.subsystems.superstructure.ElevatorWristSubsystem;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Auton {
  @Getter
  private final LoggedDashboardChooser<AutonPath> autoChooser =
      new LoggedDashboardChooser<>("Auto/Auto Chooser");

  private Sequence sequence;
  private static final Alert pathLoadAlert =
      new Alert("Error loading auton path.", AlertType.kError);

  public Auton() {
    autoChooser.addOption("LEFT", new AutonPath(Sequence.Side, "Left Group"));
    autoChooser.addOption("RIGHT", new AutonPath(Sequence.Side, "Right Group"));
    autoChooser.addOption(
        "CENTER LEFT PROCESSOR",
        new AutonPath(Sequence.Processor, "Center Start Left Group", "Center Processor Group"));
    autoChooser.addOption(
        "CENTER RIGHT PROCESSOR",
        new AutonPath(Sequence.Processor, "Center Start Right Group", "Center Processor Group"));
    autoChooser.addOption(
        "CENTER LEFT NET",
        new AutonPath(Sequence.Net, "Center Start Left Group", "Center Net Group"));
    autoChooser.addOption(
        "CENTER RIGHT NET",
        new AutonPath(Sequence.Net, "Center Start Right Group", "Center Net Group"));
    autoChooser.addOption("NONE", null);
  }

  public static enum Sequence {
    Side,

    Processor,

    Net
  }

  public static class AutonPath {
    public Sequence sequence;
    public List<String> pathList;

    public AutonPath(Sequence sequence, String... pathName) {
      this.sequence = sequence;
      pathList = Arrays.asList(pathName);
    }
  }

  public void loadPath() {
    if (autoChooser.get() != null) {
      sequence = autoChooser.get().sequence;
      pathLoadAlert.set(!AutonPathCommands.loadPathFromList(autoChooser.get().pathList));
    }
  }

  public Command execute(
      Drive myDrive,
      RollerSystem myIntake,
      RollerSystem myIntakeLow,
      ElevatorWristSubsystem myElevatorWrist,
      CoralSensor intake_sensor,
      Lights LED) {

    return new DeferredCommand(
        () -> {
          // initialization
          Command initalize = Commands.runOnce(() -> loadPath());
          Command setPose = Commands.none();
          Command autonSequence = Commands.none();

          if (Constants.currentMode == Constants.Mode.SIM) {
            setPose =
                Commands.runOnce(() -> myDrive.setPose(AutonPathCommands.getInitalStartingPose()));
          }

          switch (sequence) {
            case Side:
              autonSequence =
                  AutonSide.sequence(myIntake, myIntakeLow, myElevatorWrist, intake_sensor, LED);
              break;

            case Processor:
              autonSequence =
                  AutonProcessor.sequence(
                      myIntake, myIntakeLow, myElevatorWrist, intake_sensor, LED);
              break;

            case Net:
              autonSequence =
                  AutonNet.sequence(myIntake, myIntakeLow, myElevatorWrist, intake_sensor, LED);
              break;
          }

          // execute sequence
          return initalize.andThen(setPose).andThen(autonSequence);
        },
        Set.of(myIntake, myIntakeLow, myElevatorWrist, myDrive));
  }
}
