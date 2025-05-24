// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.Segment;
import frc.robot.subsystems.sensors.RangeSensor;
import frc.robot.subsystems.sensors.RangeSensorIO;
import frc.robot.subsystems.sensors.RangeSensorIOCANrange;
import frc.robot.subsystems.sensors.RangeSensorIOSim;
import frc.robot.subsystems.superstructure.SuperstructureConstants.Pose;
import frc.robot.subsystems.superstructure.elevator.*;
import frc.robot.subsystems.superstructure.wrist.*;
import java.util.Set;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorWristSubsystem extends SubsystemBase {
  @SuppressWarnings("unused")
  private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

  private final Lights LED;
  private final Wrist wrist;
  private final Elevator elevator;

  private final RangeSensor reefPostSensor;

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

  private final SuperstructureVisualizer measuredVisualizer =
      new SuperstructureVisualizer("Measured");
  private final SuperstructureVisualizer setpointVisualizer =
      new SuperstructureVisualizer("Setpoint");

  @SuppressWarnings("static-access")
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
        reefPostSensor =
            new RangeSensor(
                LED,
                "ReefCANrange",
                new RangeSensorIOCANrange(
                    Constants.REEFPOSTSENSOR.ID,
                    Constants.REEFPOSTSENSOR.BUS,
                    Constants.REEFPOSTSENSOR.CONFIG),
                new Segment(4, 4, 0));
        break;
      case SIM:
        wrist = new Wrist(new WristIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        reefPostSensor =
            new RangeSensor(
                LED, "ReefCANrange", new RangeSensorIOSim("Reef"), new Segment(4, 4, 0));
        break;
      default:
        wrist = new Wrist(new WristIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        reefPostSensor =
            new RangeSensor(LED, "ReefCANrange", new RangeSensorIO() {}, new Segment(4, 4, 0));
        break;
    }

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
        LED.setSolidColor(LEDColor.RED, LEDSegment.ALL);
      } else {
        LED.disableLEDs(LEDSegment.ALL);
      }
      prevIndicateToShoot = indicateToShoot;
    }

    // robot poses
    measuredVisualizer.update(
        elevator.getPositionMeters(), wrist.getPosition(), indicateToShoot, indicateToShoot);
    setpointVisualizer.update(
        elevator.getPositionMeters(), wrist.getPosition(), indicateToShoot, indicateToShoot);
  }

  public Elevator getElevator() {
    return elevator;
  }

  public Wrist getWrist() {
    return wrist;
  }

  @AutoLogOutput
  public Command setPositionCmdFailsafe(double e_goal, double w_goal) {
    return Commands.sequence(
        new ConditionalCommand(
            runOnce(() -> elevator.setPosition(SuperstructureConstants.ELEVATOR.SAFE_IN_BLOCK4))
                .andThen(new WaitUntilCommand(() -> elevator.atPosition())),
            Commands.none(),
            () -> {
              // if wrist between 3.8 and 4 and above 21 (you are at L4),
              // or
              // if elevator goal above current goal (A1 going to L3, L4 or PRENET)
              // elevator in block3 and wrist > slot3 (you are at A1),
              // go up to safe position to do wrist next
              return (wrist.getPosition() > SuperstructureConstants.WRIST.minElevate
                      && wrist.getPosition() < SuperstructureConstants.WRIST.maxElevate
                      && elevator.getPositionRotations()
                          > SuperstructureConstants.ELEVATOR.MAX_POSITION_BLOCK4)
                  || (elevator.getPositionRotations() < e_goal
                      && elevator.getPositionRotations()
                          > SuperstructureConstants.ELEVATOR.MAX_POSITION_BLOCK2
                      && elevator.getPositionRotations()
                          < SuperstructureConstants.ELEVATOR.MIN_POSITION_BLOCK4
                      && wrist.getPosition() > SuperstructureConstants.WRIST.safe);
            }),
        new ConditionalCommand( // true, wrist first, then elevator
            runOnce(() -> wrist.setPosition(SuperstructureConstants.Pose.cradle.wrist))
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
              return (wrist.getPosition() > SuperstructureConstants.WRIST.safe
                      && w_goal < SuperstructureConstants.WRIST.safe)
                  || (elevator.getPositionRotations()
                          < SuperstructureConstants.ELEVATOR.MAX_POSITION_BLOCK0
                      && e_goal > SuperstructureConstants.ELEVATOR.MAX_POSITION_BLOCK0
                      && w_goal > SuperstructureConstants.WRIST.maxElevate);
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
              return (e_goal >= elevator.getPositionRotations()
                      && w_goal <= SuperstructureConstants.WRIST.maxElevate)
                  || (w_goal <= SuperstructureConstants.WRIST.maxAtElevatorMinimum
                      && w_goal > SuperstructureConstants.WRIST.maxElevate)
                  || (w_goal > SuperstructureConstants.WRIST.maxAtElevatorMinimum
                      && ((elevator.getPositionRotations()
                                  < SuperstructureConstants.ELEVATOR.MAX_POSITION_BLOCK2
                              && elevator.getPositionRotations()
                                  > SuperstructureConstants.ELEVATOR.minAtProcessor)
                          || (elevator.getPositionRotations()
                                  < SuperstructureConstants.ELEVATOR.MAX_POSITION_BLOCK4
                              && elevator.getPositionRotations()
                                  > SuperstructureConstants.ELEVATOR.MIN_POSITION_BLOCK4)));
            }));
  }

  public Command setPositionCmd(Pose pose, int... wristSlot) {
    return setPositionCmd(pose.elevator, pose.wrist, wristSlot.length > 0 ? wristSlot[0] : 0);
  }

  public Command setPositionCmd(double e_goal, double w_goal, int... wristSlot) {
    int w_slot = wristSlot.length > 0 ? wristSlot[0] : 0;
    return new DeferredCommand(
        () -> {
          // initialization
          double e_current = elevator.getPositionRotations();
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
          if ((e_current >= SuperstructureConstants.ELEVATOR.minHighSafe
                  && e_goal < SuperstructureConstants.ELEVATOR.minHighSafe)
              && w_current < SuperstructureConstants.WRIST.safe) {
            sequence0 =
                Commands.sequence(
                    Commands.runOnce(
                        () -> Logger.recordOutput("Manipulator/Sequence0", "HIGH WRIST TO SAFE")),
                    Commands.runOnce(
                        () -> wrist.setPosition(SuperstructureConstants.WRIST.safe, w_slot)));
            // Commands.waitSeconds(.02));
          }
          ///// MOVE FROM MIN ELEVATOR TO L1, A1, OR A2 //////
          // if elevator is near zero
          // and
          // if current wrist position is less then the cradle position
          // and commanded wrist position is greater then the safe position

          if (e_current <= SuperstructureConstants.ELEVATOR.maxLowSafe
              && (w_current <= SuperstructureConstants.Pose.cradle.wrist
                  && w_goal >= SuperstructureConstants.WRIST.safe)) {
            sequence1 =
                Commands.sequence(
                    Commands.runOnce(
                        () -> Logger.recordOutput("Manipulator/Sequence1", "SLOWED WRIST")),
                    Commands.runOnce(() -> wrist.setPosition(w_goal, w_slot)),
                    Commands.runOnce(() -> elevator.setPosition(e_goal)),
                    Commands.waitUntil(
                        () ->
                            (elevator.getPositionRotations()
                                    >= SuperstructureConstants.ELEVATOR.minMidSafe)
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
          } else if (e_current <= SuperstructureConstants.ELEVATOR.maxLowSafe
              || (w_current >= SuperstructureConstants.WRIST.safe
                  && w_goal >= SuperstructureConstants.WRIST.safe)
              || (w_current >= SuperstructureConstants.WRIST.minElevate
                  && w_current <= SuperstructureConstants.WRIST.maxElevate
                  && w_goal == SuperstructureConstants.WRIST.elevate)
              || (e_current >= SuperstructureConstants.ELEVATOR.minHighSafe
                  && e_goal >= SuperstructureConstants.ELEVATOR.minHighSafe)) {
            sequence1 =
                Commands.parallel(
                    Commands.runOnce(
                        () -> Logger.recordOutput("Manipulator/Sequence1", "SIMULTANEOUS")),
                    Commands.runOnce(() -> wrist.setPosition(w_goal, w_slot)),
                    Commands.runOnce(() -> elevator.setPosition(e_goal)));
            ///// MOVE TO INTAKE POSITION //////
            // if commanded elevator position is the minimum
            // and
            // if current wrist position is near elevate position
            // or current wrist position is greater than safe position
            // or current elevator position is above high safe limit
          } else if (e_goal <= SuperstructureConstants.ELEVATOR.maxLowSafe
              && ((w_current >= SuperstructureConstants.WRIST.minElevate
                      && w_current <= SuperstructureConstants.WRIST.maxElevate)
                  || w_current >= (SuperstructureConstants.WRIST.safe - 0.05)
                  || e_current >= SuperstructureConstants.ELEVATOR.minHighSafe)) {
            if (w_current >= SuperstructureConstants.WRIST.minElevate
                && w_current <= SuperstructureConstants.WRIST.maxElevate) {
              sequence1 =
                  Commands.sequence(
                      Commands.runOnce(
                          () ->
                              Logger.recordOutput(
                                  "Manipulator/Sequence1", "THROUGH LOW SAFE (ELEVATE)")),
                      Commands.runOnce(
                          () -> wrist.setPosition(SuperstructureConstants.WRIST.elevate, w_slot)),
                      Commands.runOnce(() -> elevator.setPosition(e_goal)),
                      Commands.waitUntil(
                          () ->
                              elevator.getPositionRotations()
                                  <= SuperstructureConstants.ELEVATOR.maxLowSafe),
                      Commands.runOnce(() -> wrist.setPosition(w_goal, w_slot)),
                      Commands.waitUntil(() -> elevator.atPosition()));
            } else {
              sequence1 =
                  Commands.sequence(
                      Commands.runOnce(
                          () ->
                              Logger.recordOutput(
                                  "Manipulator/Sequence1", "THROUGH LOW SAFE (SAFE)")),
                      Commands.runOnce(
                          () -> wrist.setPosition(SuperstructureConstants.WRIST.safe, w_slot)),
                      Commands.runOnce(() -> elevator.setPosition(e_goal)),
                      Commands.waitUntil(
                          () ->
                              elevator.getPositionRotations()
                                  <= SuperstructureConstants.ELEVATOR.maxLowWristMoveFromSafe),
                      Commands.runOnce(() -> wrist.setPosition(w_goal, w_slot)));
            }
            ///// MOVE TO/FROM CORAL POSITIONS //////
            // if current wrist position is near elevate position
            // or
            // current wrist position is greater than safe position
            // or
            // current elevator position is above high safe limit
          } else if ((w_current >= SuperstructureConstants.WRIST.minElevate
                  && w_current <= SuperstructureConstants.WRIST.maxElevate)
              || w_current >= (SuperstructureConstants.WRIST.safe - 0.05)
              || e_current >= SuperstructureConstants.ELEVATOR.minHighSafe) {
            if (w_current >= SuperstructureConstants.WRIST.minElevate
                && w_current <= SuperstructureConstants.WRIST.maxElevate) {
              sequence1 =
                  Commands.sequence(
                      Commands.runOnce(
                          () ->
                              Logger.recordOutput(
                                  "Manipulator/Sequence1", "THROUGH MID SAFE (ELEVATE)")),
                      Commands.runOnce(
                          () -> wrist.setPosition(SuperstructureConstants.WRIST.elevate, w_slot)),
                      Commands.runOnce(
                          () ->
                              elevator.setPosition(
                                  (SuperstructureConstants.ELEVATOR.maxMidSafe
                                          + SuperstructureConstants.ELEVATOR.minMidSafe)
                                      / 2)),
                      Commands.waitUntil(
                          () ->
                              (elevator.getPositionRotations()
                                      <= SuperstructureConstants.ELEVATOR.maxMidSafe)
                                  && (elevator.getPositionRotations()
                                      >= SuperstructureConstants.ELEVATOR.minMidSafe)));
              if (e_goal > elevator.getPositionRotations()) {
                sequence2 =
                    Commands.sequence(
                        Commands.runOnce(
                            () ->
                                Logger.recordOutput("Manipulator/Sequence2", "Wrist -> Elevator")),
                        Commands.runOnce(() -> wrist.setPosition(w_goal, w_slot)),
                        Commands.waitUntil(
                            () -> wrist.getPosition() >= SuperstructureConstants.WRIST.safe),
                        Commands.runOnce(() -> elevator.setPosition(e_goal)));
              } else {
                sequence2 =
                    Commands.parallel(
                        Commands.runOnce(
                            () -> Logger.recordOutput("Manipulator/Sequence2", "Wrist + Elevator")),
                        Commands.runOnce(() -> wrist.setPosition(w_goal, w_slot)),
                        Commands.runOnce(() -> elevator.setPosition(e_goal)));
              }
            } else {
              sequence1 =
                  Commands.sequence(
                      Commands.runOnce(
                          () ->
                              Logger.recordOutput(
                                  "Manipulator/Sequence1", "THROUGH MID SAFE (SAFE)")),
                      Commands.runOnce(
                          () -> wrist.setPosition(SuperstructureConstants.WRIST.safe, w_slot)),
                      Commands.runOnce(
                          () ->
                              elevator.setPosition(
                                  (SuperstructureConstants.ELEVATOR.maxMidSafe
                                          + SuperstructureConstants.ELEVATOR.minMidSafe)
                                      / 2)),
                      Commands.waitUntil(
                          () ->
                              (elevator.getPositionRotations()
                                      <= SuperstructureConstants.ELEVATOR.maxMidSafe)
                                  && (elevator.getPositionRotations()
                                      >= SuperstructureConstants.ELEVATOR.minMidSafe)),
                      // Commands.waitSeconds(.05),
                      Commands.runOnce(() -> wrist.setPosition(w_goal, w_slot)),
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
  public Command setPositionElevatorCmd(double goal, boolean waitForIt) {
    return Commands.runOnce(() -> elevator.setPosition(goal))
        .andThen(Commands.waitUntil(() -> elevator.atPosition()).onlyIf(() -> waitForIt));
  }

  // no checks.  you better know what you are doing !
  public Command setPositionWristCmd(double goal, int slot, boolean waitForIt) {
    return Commands.runOnce(() -> wrist.setPosition(goal, slot))
        .andThen(Commands.waitUntil(() -> wrist.atPosition()).onlyIf(() -> waitForIt));
  }

  public Command BumpElevatorPosition(double bumpValue) {
    return Commands.runOnce(
        () -> {
          double current = elevator.getPositionRotations();
          double goal =
              Math.max(
                  Math.min(current + bumpValue, SuperstructureConstants.ELEVATOR.max),
                  SuperstructureConstants.ELEVATOR.min);
          elevator.setPosition(goal);
        });
  }

  public Command BumpWristPosition(double bumpValue) {
    return Commands.runOnce(
        () -> {
          double current = wrist.getPosition();
          double goal =
              Math.max(
                  Math.min(current + bumpValue, SuperstructureConstants.WRIST.max),
                  SuperstructureConstants.WRIST.min);
          wrist.setPosition(goal);
        });
  }

  public boolean atPosition() {
    return elevator.atPosition() && wrist.atPosition();
  }

  public void stop() {
    elevator.stop();
    wrist.stop();
  }

  public Command haltCmd() {
    return Commands.sequence(
        runOnce(() -> wrist.setPosition(wrist.getPosition())),
        runOnce(() -> elevator.setPosition(elevator.getPositionRotations())));
  }
}
