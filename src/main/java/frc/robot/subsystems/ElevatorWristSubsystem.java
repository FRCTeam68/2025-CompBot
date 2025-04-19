// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.LEDColor;
import frc.robot.Constants.LEDSegment;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.commands.ManipulatorCommands.ScoringPosition;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.superstructure.elevator.*;
import frc.robot.subsystems.superstructure.wrist.*;
import java.util.Set;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorWristSubsystem extends SubsystemBase {
  private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();
  private final Lights LED;
  private final Wrist wrist;
  private final Elevator elevator;

  @SuppressWarnings("unused")
  private final RangeSensorSubsystem reefPostSensor;

  //   @Getter @AutoLogOutput private double setpoint = 0.0;
  @Getter @AutoLogOutput private boolean reefPostDetectedRaw = false;
  @Getter @AutoLogOutput public static boolean reefPostDetected = false;
  @Getter @AutoLogOutput private boolean reefPostSensorDetected = false;
  @Getter @AutoLogOutput private double reefPostSensorDistance = 0.0;
  @Getter @AutoLogOutput private double reefPostAvgDistance = 0.0;
  private LinearFilter reefPostFilter;
  @Getter @Setter @AutoLogOutput private boolean lookingToShoot = false;
  @Getter @Setter @AutoLogOutput private boolean autoShootOn = false;
  private boolean prevIndicateToShoot = false;

  private Pose3d[] elevatorPose;
  private Pose3d wristPose;
  private Pose3d[] coralPose;
  private Pose3d[] AlgaePose;

  public ElevatorWristSubsystem(Lights LED, Supplier<Pose2d> supplier) {
    this.LED = LED;
    this.robotPoseSupplier = supplier;

    reefPostFilter = LinearFilter.movingAverage(4);
    Logger.recordOutput(
        "ElevatorWristSubsystem/reefPostHighLimit", Constants.REEFPOSTSENSOR.HIGH_LIMIT);

    switch (Constants.currentMode) {
      case REAL:
        wrist = new Wrist(new WristIOTalonFX());
        elevator = new Elevator(new ElevatorIOTalonFX());
        break;
      case SIM:
        wrist = new Wrist(new WristIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        break;
      default:
        wrist = new Wrist(new WristIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        break;
    }

    reefPostSensor = new RangeSensorSubsystem(LED, Constants.REEFPOSTSENSOR.CONFIGURATION_CONFIGS);

    zero();
  }

  private void zero() {
    elevator.zero();
  }

  public void periodic() {
    reefPostSensorDetected = reefPostSensor.isDetected();
    reefPostSensorDistance = reefPostSensor.getDistance_mm();
    reefPostAvgDistance = reefPostFilter.calculate(reefPostSensorDistance);
    reefPostDetectedRaw =
        (reefPostAvgDistance > Constants.REEFPOSTSENSOR.LOW_LIMIT)
            && (reefPostAvgDistance < Constants.REEFPOSTSENSOR.HIGH_LIMIT);
    if (Constants.bypassReefDetection) {
      reefPostDetected = true;
    } else {
      reefPostDetected = reefPostDetectedRaw;
    }
    if (Constants.tuningMode) {
      SmartDashboard.putNumber("Reef Post Sensor Avg Distance", reefPostAvgDistance);
      SmartDashboard.putNumber("Reef Post Sensor Distance", reefPostSensorDistance);
      SmartDashboard.putBoolean("Reef Post Sensor Detected", reefPostSensorDetected);
    }
    SmartDashboard.putBoolean("Reef Post Detected", reefPostDetected);

    boolean indicateToShoot = reefPostDetectedRaw && lookingToShoot;
    if (indicateToShoot != prevIndicateToShoot) {
      if (indicateToShoot) {
        LED.setColor(LEDColor.RED, LEDSegment.ALL);
      } else {
        LED.disableLEDs(LEDSegment.ALL);
      }
      prevIndicateToShoot = indicateToShoot;
    }

    // robot poses
    elevatorPose =
        new Pose3d[] {
          new Pose3d(
              0,
              0,
              Units.inchesToMeters(elevator.getPosition() * 1.751 * Math.PI),
              Rotation3d.kZero),
          new Pose3d(
              0,
              0,
              Units.inchesToMeters(elevator.getPosition() * 1.751 * Math.PI * 2),
              Rotation3d.kZero),
        };
    wristPose =
        new Pose3d(
            0.28575,
            0,
            0.411 + Units.inchesToMeters(elevator.getPosition() * 1.751 * Math.PI * 2),
            new Rotation3d(0, Units.rotationsToRadians(wrist.getPosition()), 0));
    coralPose =
        (ManipulatorCommands.getScoringPosition() == ScoringPosition.CoralL1
                    || ManipulatorCommands.getScoringPosition() == ScoringPosition.CoralL2_4)
                && ManipulatorCommands.isHavePiece()
            ? new Pose3d[] {
              new Pose3d(
                      robotPoseSupplier.get().getX() + 0.18575,
                      robotPoseSupplier.get().getY() + Units.inchesToMeters(-.4),
                      0.69 + Units.inchesToMeters(elevator.getPosition() * 1.751 * Math.PI * 2),
                      new Rotation3d(0, Units.degreesToRadians(19), 0))
                  .rotateAround(
                      new Translation3d(robotPoseSupplier.get().getTranslation())
                          .plus(wristPose.getTranslation()),
                      wristPose.getRotation())
                  .rotateAround(
                      new Translation3d(robotPoseSupplier.get().getTranslation()),
                      new Rotation3d(robotPoseSupplier.get().getRotation()))
            }
            : new Pose3d[] {};
    AlgaePose =
        (ManipulatorCommands.getScoringPosition() == ScoringPosition.Algae
                    || ManipulatorCommands.getScoringPosition() == ScoringPosition.AlgaeNet)
                && ManipulatorCommands.isHavePiece()
            ? new Pose3d[] {
              new Pose3d(
                      robotPoseSupplier.get().getX() + 0.01075,
                      robotPoseSupplier.get().getY() + Units.inchesToMeters(-.4),
                      0.445 + Units.inchesToMeters(elevator.getPosition() * 1.751 * Math.PI * 2),
                      new Rotation3d(0, Units.degreesToRadians(19), 0))
                  .rotateAround(
                      new Translation3d(robotPoseSupplier.get().getTranslation())
                          .plus(wristPose.getTranslation()),
                      wristPose.getRotation())
                  .rotateAround(
                      new Translation3d(robotPoseSupplier.get().getTranslation()),
                      new Rotation3d(robotPoseSupplier.get().getRotation()))
            }
            : new Pose3d[] {};

    Logger.recordOutput("RobotPose/Elevator", elevatorPose);
    Logger.recordOutput("RobotPose/Wrist", wristPose);
    Logger.recordOutput("RobotPose/Coral", coralPose);
    Logger.recordOutput("RobotPose/Algae", AlgaePose);
  }

  public Elevator getElevator() {
    return elevator;
  }

  public Wrist getWrist() {
    return wrist;
  }

  @AutoLogOutput
  public Command setPositionCmd(double e_goal, double w_goal) {
    return Commands.sequence(
        new ConditionalCommand(
            runOnce(() -> elevator.setPosition(Constants.ELEVATOR.SAFE_IN_BLOCK4))
                .andThen(new WaitUntilCommand(() -> elevator.atPosition())),
            Commands.none(),
            () -> {
              // if wrist between 3.8 and 4 and above 21 (you are at L4),
              // or
              // if elevator goal above current goal (A1 going to L3, L4 or PRENET)
              // elevator in block3 and wrist > slot3 (you are at A1),
              // go up to safe position to do wrist next
              return (wrist.getPosition() > Constants.WRIST.MIN_SLOT1_TO_ELEVATE
                      && wrist.getPosition() < Constants.WRIST.MAX_SLOT1_TO_ELEVATE
                      && elevator.getPosition() > Constants.ELEVATOR.MAX_POSITION_BLOCK4)
                  || (elevator.getPosition() < e_goal
                      && elevator.getPosition() > Constants.ELEVATOR.MAX_POSITION_BLOCK2
                      && elevator.getPosition() < Constants.ELEVATOR.MIN_POSITION_BLOCK4
                      && wrist.getPosition() > Constants.WRIST.MIN_POSITION_TO_CLEAR_ELEVATOR);
            }),
        new ConditionalCommand( // true, wrist first, then elevator
            runOnce(() -> wrist.setPosition(Constants.WRIST.CRADLE))
                .andThen(new WaitUntilCommand(() -> wrist.atPosition())),
            Commands.none(),
            () -> {
              // if wrist is at P1, A1, A2 position and
              //    wrist goal is less than cradle (15)
              // OR
              // if elevator nearly at 0 and
              //    elevator goal is to go past 1 and
              //   wrist past L2-L4 positions (slot1),  then
              // --> rotate to cradle position first
              // (then in next command it will do elevator, then wrist again)
              return (wrist.getPosition() > Constants.WRIST.MAX_POSITION_AT_ELEVATOR_MIN
                      && w_goal < Constants.WRIST.MIN_POSITION_TO_CLEAR_ELEVATOR)
                  || (elevator.getPosition() < Constants.ELEVATOR.MAX_POSITION_BLOCK0
                      && e_goal > Constants.ELEVATOR.MAX_POSITION_BLOCK0
                      && w_goal > Constants.WRIST.MAX_SLOT1_TO_ELEVATE);
            }),
        new ConditionalCommand( // true, wrist first, then elevator
            runOnce(() -> wrist.setPosition(w_goal))
                .andThen(new WaitUntilCommand(() -> wrist.atPosition()))
                .andThen(runOnce(() -> elevator.setPosition(e_goal)))
                .andThen(new WaitUntilCommand(() -> elevator.atPosition())),

            // false, elevator first, then wrist
            runOnce(() -> elevator.setPosition(e_goal))
                .andThen(new WaitUntilCommand(() -> elevator.atPosition()))
                .andThen(runOnce(() -> wrist.setPosition(w_goal)))
                .andThen(new WaitUntilCommand(() -> wrist.atPosition())),
            () -> {
              // if goal is to go up and going up slot1,
              // or
              // if 4 < wrist goal < 26 (e.g. PRENET)
              // or
              // if wrist goal > 14.5 (going to P1. A1, A2, PRENET OR SHOOTNET)
              // and if elevator currently between 3.4 and 5.1 (e.g. L2)
              //     or
              //     if elevator curruntly between 10.5 and 21 (e.g L3)
              return (e_goal >= elevator.getPosition()
                      && w_goal <= Constants.WRIST.MAX_SLOT1_TO_ELEVATE)
                  || (w_goal <= Constants.WRIST.MAX_POSITION_AT_ELEVATOR_MIN
                      && w_goal > Constants.WRIST.MAX_SLOT1_TO_ELEVATE)
                  || (w_goal > Constants.WRIST.MAX_POSITION_AT_ELEVATOR_MIN
                      && ((elevator.getPosition() < Constants.ELEVATOR.MAX_POSITION_BLOCK2
                              && elevator.getPosition() > Constants.ELEVATOR.MIN_POSITION_AT_P1)
                          || (elevator.getPosition() < Constants.ELEVATOR.MAX_POSITION_BLOCK4
                              && elevator.getPosition() > Constants.ELEVATOR.MIN_POSITION_BLOCK4)));
            }));
  }

  public Command setPositionCmdNew(double e_goal, double w_goal) {
    return setPositionCmdNew(e_goal, w_goal, 0);
  }

  public Command setPositionCmdNew(double e_goal, double w_goal, int wristSlot) {
    return new DeferredCommand(
        () -> {
          // initialization
          double e_current = elevator.getPosition();
          double w_current = wrist.getPosition();
          Command sequence0;
          Command sequence1;
          Command sequence2;
          Command sequenceFinal =
              Commands.parallel(
                  Commands.waitUntil(() -> wrist.atPosition()),
                  Commands.waitUntil(() -> elevator.atPosition()));
          sequence0 = Commands.runOnce(() -> Logger.recordOutput("Manipulator/Sequence0", "NULL"));
          sequence1 = Commands.runOnce(() -> Logger.recordOutput("Manipulator/Sequence1", "NULL"));
          sequence2 = Commands.runOnce(() -> Logger.recordOutput("Manipulator/Sequence2", "NULL"));
          ///// MOVE AWAY FROM SHOOTNET POSITION /////
          // if current elevator is above minimum high safe position
          // and commanded elevator is below the minimum high safe position
          // and
          // if current wrist position is less then safe position
          if ((e_current >= Constants.ELEVATOR.MIN_HIGH_SAFE
                  && e_goal < Constants.ELEVATOR.MIN_HIGH_SAFE)
              && w_current < Constants.WRIST.SAFE) {
            sequence0 =
                Commands.sequence(
                    Commands.runOnce(
                        () -> Logger.recordOutput("Manipulator/Sequence0", "HIGH WRIST TO SAFE")),
                    Commands.runOnce(() -> wrist.setPosition(Constants.WRIST.SAFE, wristSlot)));
            // Commands.waitSeconds(.02));
          }
          ///// MOVE FROM MIN ELEVATOR TO L1, A1, OR A2 //////
          // if elevator is near zero
          // and
          // if current wrist position is less then the cradle position
          // and commanded wrist position is greater then the safe position

          if (e_current <= Constants.ELEVATOR.MAX_LOW_SAFE
              && (w_current <= Constants.WRIST.CRADLE && w_goal >= Constants.WRIST.SAFE)) {
            sequence1 =
                Commands.sequence(
                    Commands.runOnce(
                        () -> Logger.recordOutput("Manipulator/Sequence1", "SLOWED WRIST")),
                    Commands.runOnce(() -> wrist.setPosition(w_goal, wristSlot)),
                    Commands.runOnce(() -> elevator.setPosition(e_goal)),
                    Commands.waitUntil(
                        () ->
                            (elevator.getPosition() >= Constants.ELEVATOR.MIN_MID_SAFE)
                                || elevator.atPosition()));

            // MAX_LOW_WRIST_MOVE_FROM_SAFE

            // Commands.runOnce(() -> wrist.setPosition(w_goal, wristSlot)));
            // Commands.waitUntil(() -> wrist.atPosition()),
            // Commands.runOnce(() -> wrist.setPosition(w_goal, wristSlot)));

            ///// MOVE FROM MIN ELEVATOR OR BETWEEN SIMILAR WRIST POSITIONS //////
            // if elevator is near zero
            // or
            // if current and commanded wrist position is greater then the safe position
            // or
            // if current and commanded wrist position is elevate position
            // or
            // if current and commanded elevator position are above minimum safe position
            //
          } else if (e_current <= Constants.ELEVATOR.MAX_LOW_SAFE
              || (w_current >= Constants.WRIST.SAFE && w_goal >= Constants.WRIST.SAFE)
              || (w_current >= Constants.WRIST.MIN_SLOT1_TO_ELEVATE
                  && w_current <= Constants.WRIST.MAX_SLOT1_TO_ELEVATE
                  && w_goal == Constants.WRIST.SLOT1_TO_ELEVATE)
              || (e_current >= Constants.ELEVATOR.MIN_HIGH_SAFE
                  && e_goal >= Constants.ELEVATOR.MIN_HIGH_SAFE)) {
            sequence1 =
                Commands.parallel(
                    Commands.runOnce(
                        () -> Logger.recordOutput("Manipulator/Sequence1", "SIMULTANEOUS")),
                    Commands.runOnce(() -> wrist.setPosition(w_goal, wristSlot)),
                    Commands.runOnce(() -> elevator.setPosition(e_goal)));
            ///// MOVE TO INTAKE POSITION //////
            // if commanded elevator position is the minimum
            // and
            // if current wrist position is near elevate position
            // or current wrist position is greater than safe position
            // or current elevator position is above high safe limit
          } else if (e_goal <= Constants.ELEVATOR.MAX_LOW_SAFE
              && ((w_current >= Constants.WRIST.MIN_SLOT1_TO_ELEVATE
                      && w_current <= Constants.WRIST.MAX_SLOT1_TO_ELEVATE)
                  || w_current >= (Constants.WRIST.SAFE - Constants.WRIST.ERROR)
                  || e_current >= Constants.ELEVATOR.MIN_HIGH_SAFE)) {
            if (w_current >= Constants.WRIST.MIN_SLOT1_TO_ELEVATE
                && w_current <= Constants.WRIST.MAX_SLOT1_TO_ELEVATE) {
              sequence1 =
                  Commands.sequence(
                      Commands.runOnce(
                          () ->
                              Logger.recordOutput(
                                  "Manipulator/Sequence1", "THROUGH LOW SAFE (ELEVATE)")),
                      Commands.runOnce(
                          () -> wrist.setPosition(Constants.WRIST.SLOT1_TO_ELEVATE, wristSlot)),
                      Commands.runOnce(() -> elevator.setPosition(e_goal)),
                      Commands.waitUntil(
                          () -> elevator.getPosition() <= Constants.ELEVATOR.MAX_LOW_SAFE),
                      Commands.runOnce(() -> wrist.setPosition(w_goal, wristSlot)),
                      Commands.waitUntil(() -> elevator.atPosition()));
            } else {
              sequence1 =
                  Commands.sequence(
                      Commands.runOnce(
                          () ->
                              Logger.recordOutput(
                                  "Manipulator/Sequence1", "THROUGH LOW SAFE (SAFE)")),
                      Commands.runOnce(() -> wrist.setPosition(Constants.WRIST.SAFE, wristSlot)),
                      Commands.runOnce(() -> elevator.setPosition(e_goal)),
                      Commands.waitUntil(
                          () ->
                              elevator.getPosition()
                                  <= Constants.ELEVATOR.MAX_LOW_WRIST_MOVE_FROM_SAFE),
                      Commands.runOnce(() -> wrist.setPosition(w_goal, wristSlot)));
            }
            ///// MOVE TO/FROM CORAL POSITIONS //////
            // if current wrist position is near elevate position
            // or
            // current wrist position is greater than safe position
            // or
            // current elevator position is above high safe limit
          } else if ((w_current >= Constants.WRIST.MIN_SLOT1_TO_ELEVATE
                  && w_current <= Constants.WRIST.MAX_SLOT1_TO_ELEVATE)
              || w_current >= (Constants.WRIST.SAFE - Constants.WRIST.ERROR)
              || e_current >= Constants.ELEVATOR.MIN_HIGH_SAFE) {
            if (w_current >= Constants.WRIST.MIN_SLOT1_TO_ELEVATE
                && w_current <= Constants.WRIST.MAX_SLOT1_TO_ELEVATE) {
              sequence1 =
                  Commands.sequence(
                      Commands.runOnce(
                          () ->
                              Logger.recordOutput(
                                  "Manipulator/Sequence1", "THROUGH MID SAFE (ELEVATE)")),
                      Commands.runOnce(
                          () -> wrist.setPosition(Constants.WRIST.SLOT1_TO_ELEVATE, wristSlot)),
                      Commands.runOnce(
                          () ->
                              elevator.setPosition(
                                  ((Constants.ELEVATOR.MAX_MID_SAFE
                                              - Constants.ELEVATOR.MIN_MID_SAFE)
                                          / 2)
                                      + Constants.ELEVATOR.MIN_MID_SAFE)),
                      Commands.waitUntil(
                          () ->
                              (elevator.getPosition() <= Constants.ELEVATOR.MAX_MID_SAFE)
                                  && (elevator.getPosition() >= Constants.ELEVATOR.MIN_MID_SAFE)));
              if (e_goal > elevator.getPosition()) {
                sequence2 =
                    Commands.sequence(
                        Commands.runOnce(
                            () ->
                                Logger.recordOutput("Manipulator/Sequence2", "Wrist -> Elevator")),
                        Commands.runOnce(() -> wrist.setPosition(w_goal, wristSlot)),
                        Commands.waitUntil(() -> wrist.getPosition() >= Constants.WRIST.SAFE),
                        Commands.runOnce(() -> elevator.setPosition(e_goal)));
              } else {
                sequence2 =
                    Commands.parallel(
                        Commands.runOnce(
                            () -> Logger.recordOutput("Manipulator/Sequence2", "Wrist + Elevator")),
                        Commands.runOnce(() -> wrist.setPosition(w_goal, wristSlot)),
                        Commands.runOnce(() -> elevator.setPosition(e_goal)));
              }
            } else {
              sequence1 =
                  Commands.sequence(
                      Commands.runOnce(
                          () ->
                              Logger.recordOutput(
                                  "Manipulator/Sequence1", "THROUGH MID SAFE (SAFE)")),
                      Commands.runOnce(() -> wrist.setPosition(Constants.WRIST.SAFE, wristSlot)),
                      Commands.runOnce(
                          () ->
                              elevator.setPosition(
                                  ((Constants.ELEVATOR.MAX_MID_SAFE
                                              - Constants.ELEVATOR.MIN_MID_SAFE)
                                          / 2)
                                      + Constants.ELEVATOR.MIN_MID_SAFE)),
                      Commands.waitUntil(
                          () ->
                              (elevator.getPosition() <= Constants.ELEVATOR.MAX_MID_SAFE)
                                  && (elevator.getPosition() >= Constants.ELEVATOR.MIN_MID_SAFE)),
                      // Commands.waitSeconds(.05),
                      Commands.runOnce(() -> wrist.setPosition(w_goal, wristSlot)),
                      Commands.runOnce(() -> elevator.setPosition(e_goal)));
            }
          } else { // fallback to failsafe sequence
            sequence1 =
                Commands.sequence(
                    Commands.runOnce(
                        () -> Logger.recordOutput("Manipulator/Sequence1", "FAILSAFE")),
                    setPositionCmd(e_goal, w_goal));
          }
          // execute sequence
          return sequence0.andThen(sequence1).andThen(sequence2).andThen(sequenceFinal);
        },
        Set.of(this));
  }

  // no checks.  you better know what you are doing !
  public Command setPositionElevatorCmd(double e_goal, boolean waitForIt) {
    return Commands.sequence(
        runOnce(() -> elevator.setPosition(e_goal)),
        new ConditionalCommand(
            new WaitUntilCommand(() -> elevator.atPosition()),
            Commands.none(),
            () -> {
              return waitForIt;
            }));
  }

  // no checks.  you better know what you are doing !
  public Command setPositionWristCmd(double w_goal, int slot, boolean waitForIt) {
    return Commands.sequence(
        runOnce(() -> wrist.setPosition(w_goal, slot)),
        new ConditionalCommand(
            new WaitUntilCommand(() -> wrist.atPosition()),
            Commands.none(),
            () -> {
              return waitForIt;
            }));
  }

  public boolean atPosition() {
    return elevator.atPosition() && wrist.atPosition();
  }

  public Command BumpElevatorPosition(double bumpValue) {
    // double elevatorNow = elevator.getPosition();
    return new ConditionalCommand(
        runOnce(() -> elevator.setPosition(elevator.getPosition() + bumpValue)),
        Commands.none(),
        () -> {
          double elevatorNow = elevator.getPosition();
          return elevatorNow + bumpValue > Constants.ELEVATOR.MIN_POSITION - 0.2
              && elevatorNow + bumpValue < Constants.ELEVATOR.MAX_POSITION;
        });
  }

  public Command BumpWristPosition(double bumpValue) {
    // double elevatorNow = elevator.getPosition();
    return new ConditionalCommand(
        runOnce(() -> wrist.setPosition(wrist.getPosition() + bumpValue)),
        Commands.none(),
        () -> {
          double wristNow = wrist.getPosition();
          double elevatorNow = elevator.getPosition();
          return (wristNow + bumpValue > Constants.WRIST.MIN_POSITION - 0.2
                  && wristNow + bumpValue < Constants.WRIST.MAX_POSITION_AT_ELEVATOR_MIN)
              || (elevatorNow >= Constants.ELEVATOR.MIN_POSITION_AT_P1 - 0.2
                  && wristNow + bumpValue < Constants.WRIST.MAX_POSITION_AT_P1);
        });
  }

  public Command staticElevatorCharacterization(double outputRampRate) {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              // stopProfile = true;
              timer.restart();
            },
            () -> {
              state.characterizationOutput = outputRampRate * timer.get();
              elevator.setVolts(state.characterizationOutput);
              Logger.recordOutput(
                  "Elevator/StaticCharacterizationOutput", state.characterizationOutput);
            })
        .until(() -> elevator.getPosition() >= Constants.ELEVATOR.MAX_POSITION - 1)
        .finallyDo(
            () -> {
              // stopProfile = false;
              timer.stop();
              Logger.recordOutput("Elevator/CharacterizationOutput", state.characterizationOutput);
            });
  }

  public Command staticWristCharacterization(double outputRampRate) {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              // stopProfile = true;
              timer.restart();
            },
            () -> {
              state.characterizationOutput = outputRampRate * timer.get();
              elevator.setVolts(state.characterizationOutput);
              Logger.recordOutput(
                  "Wrist/StaticCharacterizationOutput", state.characterizationOutput);
            })
        .until(() -> elevator.getPosition() >= Constants.WRIST.MAX_POSITION_AT_ELEVATOR_MIN)
        .finallyDo(
            () -> {
              // stopProfile = false;
              timer.stop();
              Logger.recordOutput("Wrist/CharacterizationOutput", state.characterizationOutput);
            });
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }

  public void stop() {
    elevator.stop();
    wrist.stop();
  }

  public Command haltCmd() {
    return Commands.sequence(
        runOnce(() -> wrist.setPosition(wrist.getPosition())),
        runOnce(() -> elevator.setPosition(elevator.getPosition())));
  }
}
