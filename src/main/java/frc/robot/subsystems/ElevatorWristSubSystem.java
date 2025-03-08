// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIOTalonFX;
import java.util.Set;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorWristSubSystem extends SubsystemBase {

  private final RollerSystem wrist;
  private final RollerSystem elevator;
  private final RollerSystem elevatorFollower;
  private final RangeSensorSubSystem ElevatorSensor;
  private final CANcoder wristCANcoder;

  @Getter @AutoLogOutput private double setpoint = 0.0;
  @Getter @AutoLogOutput private double elevatorHeight = 0.0;
  @Getter @AutoLogOutput private double wristAngle = 0.0;
  @Getter @AutoLogOutput private double e_goal = 0;
  @Getter @AutoLogOutput private double w_goal = 0;
  private double e_bump_goal = 0;
  private double w_bump_goal = 0;

  public ElevatorWristSubSystem() {

    wrist =
        new RollerSystem(
            "Wrist",
            new RollerSystemIOTalonFX(
                Constants.WRIST.CANID, Constants.WRIST.CANBUS, 80, true, 0, false, true, 1));
    // init tunables in the parent roller system
    wrist.setPID(Constants.WRIST.SLOT0_CONFIGS);
    wrist.setMotionMagic(Constants.WRIST.MOTIONMAGIC_CONFIGS);
    wrist.setAtSetpointBand(.3);
    wrist.setPieceCurrentThreshold(
        40); // does not have a piece but might want to use to detect overrun limits?

    wristCANcoder = new CANcoder(Constants.WRIST.CANCODER_CANID, Constants.WRIST.CANBUS);
    var cancoderConfig = new CANcoderConfiguration();
    // cancoderConfig.MagnetSensor.MagnetOffset = offset.getRotations();
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = Constants.WRIST.CANCODER_OFFSET;
    tryUntilOk(5, () -> wristCANcoder.getConfigurator().apply(cancoderConfig));

    elevator =
        new RollerSystem(
            "Elevator",
            new RollerSystemIOTalonFX(
                Constants.ELEVATOR.RIGHT_CANID,
                Constants.ELEVATOR.CANBUS,
                80,
                false,
                0,
                false,
                true,
                1));
    // init tunables in the parent roller system
    elevator.setPID(Constants.ELEVATOR.SLOT0_CONFIGS);
    elevator.setMotionMagic(Constants.ELEVATOR.MOTIONMAGIC_CONFIGS);
    elevator.setAtSetpointBand(.3);
    elevator.setPieceCurrentThreshold(
        40); // does not have a piece but might want to use to detect overrun limits?
    elevatorFollower =
        new RollerSystem(
            "ElevatorFollower",
            new RollerSystemIOTalonFX(
                Constants.ELEVATOR.LEFT_CANID,
                Constants.ELEVATOR.CANBUS,
                80,
                false,
                Constants.ELEVATOR.RIGHT_CANID,
                true,
                true,
                1));

    ElevatorSensor =
        new RangeSensorSubSystem(
            "ElevatorHeight",
            Constants.ELEVATOR_SENSOR.CANID,
            Constants.ELEVATOR_SENSOR.CANBUS,
            Constants.ELEVATOR_SENSOR.THRESHOLD);

    // this should account for wrist not starting in zero position
    // wrist.setPosition(
    //     wristCANcoder.getPosition().getValueAsDouble() * Constants.WRIST.CANCODER_FACTOR);
    // elevator.setPosition(0);

    wristAngle = wristCANcoder.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("WristAngle", wristAngle);
    SmartDashboard.putBoolean("Wrist Zeroed", wristAngle < 0.001);

    zero();
  }

  public void zero() {
    wrist.zero();
    elevator.zero();
  }

  public void periodic() {
    elevatorHeight = elevator.getPosition();
    SmartDashboard.putNumber("ElevatorHieght", elevatorHeight);
    SmartDashboard.putBoolean("Elevator at Zero", elevatorHeight < 96);

    wristAngle = wristCANcoder.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("WristAngle", wristAngle);
    // SmartDashboard.putBoolean("Wrist Zeroed", wristAngle < 0.001);

    // robot poses
    Logger.recordOutput(
        "RobotPose/Elevator Stage 1",
        new Pose3d[] {
          new Pose3d(
              0, 0, Units.inchesToMeters(elevator.getPosition() / 5 * 5.5), new Rotation3d(0, 0, 0))
        });
    Logger.recordOutput(
        "RobotPose/Elevator Stage 2",
        new Pose3d[] {
          new Pose3d(
              0,
              0,
              Units.inchesToMeters((elevator.getPosition() / 5 * 5.5) * 2),
              new Rotation3d(0, 0, 0))
        });
    Logger.recordOutput(
        "RobotPose/Wrist",
        new Pose3d[] {
          new Pose3d(
              0.28575,
              0,
              0.411 + Units.inchesToMeters((elevator.getPosition() / 5 * 5.5) * 2),
              new Rotation3d(0, wrist.getPosition() / 25 * (12 / 30), 0))
        });
  }

  public RollerSystem getElevator() {
    return elevator;
  }

  public RollerSystem getWrist() {
    return wrist;
  }

  @AutoLogOutput
  public Command setPositionCmd(double e_goal, double w_goal) {
    return Commands.sequence(
        Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.red)),
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
    return new DeferredCommand(
        () -> {
          // initialization
          double e_current = elevator.getPosition();
          double w_current = wrist.getPosition();
          Command sequence0;
          Command sequence1;
          Command sequence2;
          Command sequenceFinal =
              Commands.sequence(
                  Commands.waitUntil(() -> wrist.atPosition()),
                  Commands.waitUntil(() -> elevator.atPosition()));
          Command ledBegin =
              Commands.runOnce(() -> LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4));
          Command ledEnd = Commands.runOnce(() -> LEDSegment.all.setColor(LightsSubsystem.orange));
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
                    Commands.runOnce(() -> wrist.setPosition(Constants.WRIST.SAFE)));
            // Commands.waitSeconds(.02));
          }
          ///// MOVE FROM MIN ELEVATOR OR BETWEEN SIMILAR WRIST POSITIONS //////
          // if elevator is near zero
          // or
          // if current and commanded wrist position is greater then the safe position
          // or
          // if current and commanded wrist position is elevate position
          // or
          // if current and commanded elevator position are above minimum safe position
          if (e_current <= Constants.ELEVATOR.MAX_LOW_SAFE
              || (w_current >= Constants.WRIST.SAFE && w_goal >= Constants.WRIST.SAFE)
              || (w_current >= Constants.WRIST.MIN_SLOT1_TO_ELEVATE
                  && w_current <= Constants.WRIST.MAX_SLOT1_TO_ELEVATE
                  && w_goal == Constants.WRIST.SLOT1_TO_ELEVATE)
              || (e_current >= Constants.ELEVATOR.MIN_HIGH_SAFE
                  && e_goal >= Constants.ELEVATOR.MIN_HIGH_SAFE)) {
            if (w_goal <= Constants.WRIST.CRADLE
                || w_current >= Constants.WRIST.CRADLE - Constants.WRIST.ERROR) {
              sequence1 =
                  Commands.sequence(
                      Commands.runOnce(
                          () -> Logger.recordOutput("Manipulator/Sequence1", "SIMULTANEOUS")),
                      Commands.runOnce(() -> wrist.setPosition(w_goal)),
                      Commands.runOnce(() -> elevator.setPosition(e_goal)));
            } else {
              sequence1 =
                  Commands.sequence(
                      Commands.runOnce(
                          () -> Logger.recordOutput("Manipulator/Sequence1", "WRIST -> ELEVATOR")),
                      Commands.runOnce(() -> wrist.setPosition(w_goal)),
                      Commands.waitSeconds(.3),
                      Commands.runOnce(() -> elevator.setPosition(e_goal)));
            }
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
                      // algaeintake.setSpeedCmd(5),
                      Commands.runOnce(() -> wrist.setPosition(Constants.WRIST.SLOT1_TO_ELEVATE)),
                      Commands.runOnce(() -> elevator.setPosition(e_goal)),
                      Commands.waitUntil(
                          () -> elevator.getPosition() <= Constants.ELEVATOR.MAX_LOW_SAFE),
                      Commands.runOnce(() -> wrist.setPosition(w_goal)),
                      Commands.waitUntil(() -> elevator.atPosition())
                      // algaeintake.setSpeedCmd(0),
                      );
            } else {
              sequence1 =
                  Commands.sequence(
                      Commands.runOnce(
                          () ->
                              Logger.recordOutput(
                                  "Manipulator/Sequence1", "THROUGH LOW SAFE (SAFE)")),
                      Commands.runOnce(() -> wrist.setPosition(Constants.WRIST.SAFE)),
                      Commands.runOnce(() -> elevator.setPosition(e_goal)),
                      Commands.waitUntil(
                          () ->
                              elevator.getPosition()
                                  <= Constants.ELEVATOR.MAX_LOW_WRIST_MOVE_FROM_SAFE),
                      Commands.runOnce(() -> wrist.setPosition(w_goal)));
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
                      Commands.runOnce(() -> wrist.setPosition(Constants.WRIST.SLOT1_TO_ELEVATE)),
                      Commands.runOnce(
                          () ->
                              elevator.setPosition(
                                  Constants.ELEVATOR.MAX_MID_SAFE
                                      - Constants.ELEVATOR.MIN_MID_SAFE
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
                        Commands.runOnce(() -> wrist.setPosition(w_goal)),
                        Commands.waitUntil(() -> wrist.getPosition() >= Constants.WRIST.SAFE),
                        Commands.runOnce(() -> elevator.setPosition(e_goal)));
              } else {
                sequence2 =
                    Commands.parallel(
                        Commands.runOnce(
                            () -> Logger.recordOutput("Manipulator/Sequence2", "Wrist + Elevator")),
                        Commands.runOnce(() -> wrist.setPosition(w_goal)),
                        Commands.runOnce(() -> elevator.setPosition(e_goal)));
              }
            } else {
              sequence1 =
                  Commands.sequence(
                      Commands.runOnce(
                          () ->
                              Logger.recordOutput(
                                  "Manipulator/Sequence1", "THROUGH MID SAFE (SAFE)")),
                      Commands.runOnce(() -> wrist.setPosition(Constants.WRIST.SAFE)),
                      Commands.runOnce(
                          () ->
                              elevator.setPosition(
                                  Constants.ELEVATOR.MAX_MID_SAFE
                                      - Constants.ELEVATOR.MIN_MID_SAFE
                                      + Constants.ELEVATOR.MIN_MID_SAFE)),
                      Commands.waitUntil(
                          () ->
                              (elevator.getPosition() <= Constants.ELEVATOR.MAX_MID_SAFE)
                                  && (elevator.getPosition() >= Constants.ELEVATOR.MIN_MID_SAFE)),
                      Commands.waitSeconds(.3),
                      Commands.runOnce(() -> wrist.setPosition(w_goal)),
                      Commands.runOnce(() -> elevator.setPosition(e_goal)));
            }
          } else { // fallback to failsafe sequence
            // return setPositionCmd(e_goal, w_goal);
            Commands.sequence(
                Commands.runOnce(() -> Logger.recordOutput("Manipulator/Sequence1", "FAILSAFE")),
                Commands.waitSeconds(5));
          }
          // execute sequence
          return ledBegin
              .andThen(sequence0)
              .andThen(sequence1)
              .andThen(sequence2)
              .andThen(sequenceFinal)
              .andThen(ledEnd);
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
  public Command setPositionWristCmd(double w_goal, boolean waitForIt) {
    return Commands.sequence(
        runOnce(() -> wrist.setPosition(w_goal)),
        new ConditionalCommand(
            new WaitUntilCommand(() -> wrist.atPosition()),
            Commands.none(),
            () -> {
              return waitForIt;
            }));
  }

  // public double getElevatorHeight() {
  //   return elevatorHeight;
  // }

  // public double getWristAngle() {
  //   return wristAngle;
  // }

  public boolean atPosition() {
    return elevator.atPosition() && wrist.atPosition();
  }

  public Command BumpElevatorPosition(double bumpValue) {
    // initialization
    e_bump_goal = elevator.getPosition() + bumpValue;
    // check limits
    if (e_bump_goal < Constants.ELEVATOR.MIN_POSITION) {
      e_bump_goal = Constants.ELEVATOR.MIN_POSITION;
    } else if (e_goal > Constants.ELEVATOR.MAX_POSITION) {
      e_bump_goal = Constants.ELEVATOR.MAX_POSITION;
    }
    // coral L1 offset
    if (Constants.WRIST.POSITION_SCORING_ELEMENT == "CoralL1") {
      Constants.ELEVATOR.L1_OFFSET = Constants.ELEVATOR.L1_OFFSET + bumpValue;
      Logger.recordOutput("Roller/Elevator/L1 Offset", Constants.ELEVATOR.L1_OFFSET);
    }
    // set position
    return runOnce(() -> elevator.setPosition(e_bump_goal));
  }

  public Command BumpWristPosition(double bumpValue) {
    // initialization
    w_bump_goal = wrist.getPosition() + bumpValue;
    // check limits
    if (w_bump_goal < Constants.WRIST.MIN_POSITION) {
      w_bump_goal = Constants.WRIST.MIN_POSITION;
    } else if (w_goal > Constants.WRIST.MAX_POSITION) {
      w_bump_goal = Constants.WRIST.MAX_POSITION;
    }
    // coral L1 offset
    if (Constants.WRIST.POSITION_SCORING_ELEMENT == "CoralL1") {
      Constants.WRIST.L1_OFFSET = Constants.WRIST.L1_OFFSET + bumpValue;
      Logger.recordOutput("Roller/Wrist/L1 Offset", Constants.WRIST.L1_OFFSET);
    }
    // set position
    return runOnce(() -> wrist.setPosition(w_bump_goal));
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
