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
import frc.robot.commands.auton.autonSequences.*;
import frc.robot.subsystems.ElevatorWristSubsystem;
import frc.robot.subsystems.RangeSensorSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.rollers.RollerSystem;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

public class Auton {
  private final Drive m_drive;

  private Sequence sequence;
  private static final Alert pathLoadAlert = new Alert("Error loading path.", AlertType.kError);

  public Auton(Drive drive) {
    m_drive = drive;
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

  public void loadPath(AutonPath autonPath) {
    sequence = autonPath.sequence;
    pathLoadAlert.set(!AutonPathCommands.loadPathFromList(autonPath.pathList));
  }

  public Command execute(
      RollerSystem myIntake,
      RollerSystem myIntakeLow,
      ElevatorWristSubsystem myElevatorWrist,
      RangeSensorSubsystem intake_sensor,
      Lights LED) {

    return new DeferredCommand(
        () -> {
          // initialization
          Command setPose = Commands.none();
          Command autonSequence = Commands.none();

          if (Constants.currentMode == Constants.Mode.SIM) {
            setPose =
                Commands.runOnce(() -> m_drive.setPose(AutonPathCommands.getInitalStartingPose()));
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
          return setPose.andThen(autonSequence);
        },
        Set.of(myIntake, myIntakeLow, myElevatorWrist, m_drive));
  }
}
