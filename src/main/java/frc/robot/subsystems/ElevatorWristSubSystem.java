// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
// import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIOTalonFX;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class ElevatorWristSubSystem extends SubsystemBase {

  private final RollerSystem wrist;
  private final RollerSystem elevator;
  private final RollerSystem elevatorFollower;

  @Getter @AutoLogOutput private double setpoint = 0.0;

  public ElevatorWristSubSystem() {

    wrist =
        new RollerSystem(
            "Wrist",
            new RollerSystemIOTalonFX(
                Constants.WRIST.CANID, Constants.WRIST.CANBUS, 40, false, 0, false, true, 1));
    // init tunables in the parent roller system
    wrist.setPID(Constants.WRIST.SLOT0_CONFIGS);
    wrist.setMotionMagic(Constants.WRIST.MOTIONMAGIC_CONFIGS);
    wrist.setAtSetpointBand(.3);

    elevator =
        new RollerSystem(
            "Elevator",
            new RollerSystemIOTalonFX(
                Constants.ELEVATOR.LEFT_CANID,
                Constants.ELEVATOR.CANBUS,
                40,
                false,
                0,
                false,
                false,
                1));
    // init tunables in the parent roller system
    elevator.setPID(Constants.ELEVATOR.SLOT0_CONFIGS);
    elevator.setMotionMagic(Constants.ELEVATOR.MOTIONMAGIC_CONFIGS);
    elevator.setAtSetpointBand(.3);
    elevatorFollower =
        new RollerSystem(
            "ElevatorFollower",
            new RollerSystemIOTalonFX(
                Constants.ELEVATOR.RIGHT_CANID,
                Constants.ELEVATOR.CANBUS,
                40,
                true,
                Constants.ELEVATOR.LEFT_CANID,
                true,
                true,
                1));
    // init tunables in the parent roller system
    elevatorFollower.setPID(Constants.ELEVATOR.SLOT0_CONFIGS);
    elevatorFollower.setMotionMagic(Constants.ELEVATOR.MOTIONMAGIC_CONFIGS);
    elevatorFollower.setAtSetpointBand(.3);

    wrist.setPosition(0);
    elevator.setPosition(0);

    zero();
  }

  public void zero() {
    wrist.zero();
    elevator.zero();
  }

  public void periodic() {
    double elevatorNow = elevator.getPosition();
    double wristNow = wrist.getPosition();

    // stop wrist if beyond limits
    if (wristNow < Constants.WRIST.MIN_POSITION - 0.25) {
      wrist.stop();
      // wrist.setPosition(Constants.WRIST.MIN_POSITION);
      System.out.println("********** min wrist angle: " + Constants.WRIST.MIN_POSITION);
    } else if (wristNow > Constants.WRIST.MAX_POSITION_AT_P1) {
      wrist.stop();
      // wrist.setPosition(Constants.WRIST.MAX_POSITION_AT_P1);
      System.out.println("********** max wrist angle: " + Constants.WRIST.MAX_POSITION_AT_P1);
    } else if (wristNow < Constants.WRIST.MIN_POSITION_TO_CLEAR_ELEVATOR
        && elevatorNow > Constants.ELEVATOR.MAX_POSITION_WRIST_NOT_CLEAR) {
      // stop the elevator
      wrist.stop();
      elevator.stop();
      // wrist.setPosition(Constants.WRIST.MIN_POSITION_TO_CLEAR_ELEVATOR);
      // elevator.setPosition(Constants.ELEVATOR.MAX_POSITION_WRIST_NOT_CLEAR);
      System.out.println(
          "********** wrist angle cannot be less than: "
              + Constants.WRIST.MIN_POSITION_TO_CLEAR_ELEVATOR
              + ", when elevator above height: "
              + Constants.ELEVATOR.MAX_POSITION_WRIST_NOT_CLEAR);
    }

    // stop elevator if beyond limits
    if (elevatorNow < Constants.ELEVATOR.MIN_POSITION - 0.25) {
      // stop the elevator at minimum
      elevator.stop();
      // elevator.setPosition(Constants.ELEVATOR.MIN_POSITION);
      System.out.println("********** min elevator height: " + Constants.ELEVATOR.MIN_POSITION);
    } else if (elevatorNow > Constants.ELEVATOR.MAX_POSITION) {
      elevator.stop();
      // elevator.setPosition(Constants.ELEVATOR.MAX_POSITION);
      System.out.println("********** max elevator height: " + Constants.ELEVATOR.MAX_POSITION);
    } else if (wristNow > Constants.WRIST.MAX_POSITION_AT_P1
        && elevatorNow < Constants.ELEVATOR.MIN_POSITION_AT_P1) {
      elevator.stop();
      // elevator.setPosition(Constants.ELEVATOR.MIN_POSITION_AT_P1);
      System.out.println(
          "********** min elelvator height angle: "
              + Constants.ELEVATOR.MIN_POSITION_AT_P1
              + ", when wrist at angle: "
              + Constants.WRIST.MAX_POSITION_AT_P1);
    }
  }

  @AutoLogOutput
  public Command setPositionCmd(double e_goal, double w_goal) {
    // return runOnce(() -> LEDSegment.side1.setBandAnimation(LightsSubsystem.green, .5))
    //     .andThen(
    return new ConditionalCommand( // true, wrist first, then elevator
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
          // if goal is to go up or going and not going to processor position, do wrist
          // first
          return e_goal >= elevator.getPosition()
              || w_goal <= Constants.WRIST.MAX_POSITION_AT_ELEVATOR_MIN;
        });
    // .andThen(runOnce(() -> LEDSegment.side1.setColor(LightsSubsystem.green)));
  }

  public boolean atPosition() {
    return elevator.atPosition() && wrist.atPosition();
  }

  public Command BumpElevatorPosition(double bumpValue) {
    return runOnce(() -> elevator.setPosition(elevator.getPosition() + bumpValue));
  }

  public Command BumpWristPosition(double bumpValue) {
    return runOnce(() -> wrist.setPosition(wrist.getPosition() + bumpValue));
  }
}
