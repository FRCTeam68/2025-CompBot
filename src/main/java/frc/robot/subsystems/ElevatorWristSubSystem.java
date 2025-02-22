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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
// import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIOTalonFX;
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

    SmartDashboard.putNumber("ElevatorHieght", 0);

    wrist.setPosition(0);
    elevator.setPosition(0);

    zero();
  }

  public void zero() {
    wrist.zero();
    elevator.zero();
  }

  public void periodic() {
    elevatorHeight = ElevatorSensor.getDistance_mm();
    SmartDashboard.putNumber("ElevatorHieght", elevatorHeight);
    wristAngle = wristCANcoder.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("WristAngle", wristAngle);

    // double elevatorNow = elevator.getPosition();
    // double wristNow = wrist.getPosition();

    // // stop wrist if beyond limits
    // if (wristNow < Constants.WRIST.MIN_POSITION - 0.2) {
    //   wrist.stop();
    //   // wrist.setPosition(Constants.WRIST.MIN_POSITION);
    //   System.out.println("********** min wrist angle: " + Constants.WRIST.MIN_POSITION);
    // } else if (wristNow > Constants.WRIST.MAX_POSITION_AT_P1) {
    //   wrist.stop();
    //   // wrist.setPosition(Constants.WRIST.MAX_POSITION_AT_P1);
    //   System.out.println("********** max wrist angle: " + Constants.WRIST.MAX_POSITION_AT_P1);
    // } else if (wristNow < Constants.WRIST.MIN_POSITION_TO_CLEAR_ELEVATOR
    //     && elevatorNow > Constants.ELEVATOR.MAX_POSITION_WRIST_NOT_CLEAR) {
    //   // stop the elevator
    //   wrist.stop();
    //   elevator.stop();
    //   // wrist.setPosition(Constants.WRIST.MIN_POSITION_TO_CLEAR_ELEVATOR);
    //   // elevator.setPosition(Constants.ELEVATOR.MAX_POSITION_WRIST_NOT_CLEAR);
    //   System.out.println(
    //       "********** wrist angle cannot be less than: "
    //           + Constants.WRIST.MIN_POSITION_TO_CLEAR_ELEVATOR
    //           + ", when elevator above height: "
    //           + Constants.ELEVATOR.MAX_POSITION_WRIST_NOT_CLEAR);
    // }

    // // stop elevator if beyond limits
    // if (elevatorNow < Constants.ELEVATOR.MIN_POSITION - 0.25) {
    //   // stop the elevator at minimum
    //   elevator.stop();
    //   // elevator.setPosition(Constants.ELEVATOR.MIN_POSITION);
    //   System.out.println("********** min elevator height: " + Constants.ELEVATOR.MIN_POSITION);
    // } else if (elevatorNow > Constants.ELEVATOR.MAX_POSITION) {
    //   elevator.stop();
    //   // elevator.setPosition(Constants.ELEVATOR.MAX_POSITION);
    //   System.out.println("********** max elevator height: " + Constants.ELEVATOR.MAX_POSITION);
    // } else if (wristNow > Constants.WRIST.MAX_POSITION_AT_P1
    //     && elevatorNow < Constants.ELEVATOR.MIN_POSITION_AT_P1) {
    //   elevator.stop();
    //   // elevator.setPosition(Constants.ELEVATOR.MIN_POSITION_AT_P1);
    //   System.out.println(
    //       "********** min elelvator height angle: "
    //           + Constants.ELEVATOR.MIN_POSITION_AT_P1
    //           + ", when wrist at angle: "
    //           + Constants.WRIST.MAX_POSITION_AT_P1);
    // }
  }

  @AutoLogOutput
  public Command setPositionCmd(double e_goal, double w_goal) {
    return Commands.sequence(
        new ConditionalCommand( // true, wrist first, then elevator
            runOnce(() -> wrist.setPosition(Constants.WRIST.CRADLE))
                .andThen(new WaitUntilCommand(() -> wrist.atPosition())),
            Commands.none(),
            () -> {
              // if wrist is at P1 position   OR
              // elevator nearly at 0 and
              // elevator goal is to go past 1 and
              // wrist past L1-L4 positions,  then
              // --> rotate to cradle position first
              // (then in next command it will do elevator, then wrist again)
              return wrist.getPosition() > Constants.WRIST.MAX_POSITION_AT_ELEVATOR_MIN
                  || (elevator.getPosition() < Constants.ELEVATOR.MAX_POSITION_WRIST_NOT_CLEAR
                      && e_goal > Constants.ELEVATOR.MAX_POSITION_WRIST_NOT_CLEAR
                      && w_goal > Constants.WRIST.L2);
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
              // if goal is to go up and wrist goal will not hit bumper, do wrist first
              // or
              return (e_goal >= elevator.getPosition() && w_goal <= Constants.WRIST.L2)
                  || (w_goal <= Constants.WRIST.MAX_POSITION_AT_ELEVATOR_MIN
                      && w_goal > Constants.WRIST.L2);
            }));
  }

  // // go to safe position everytime
  // runOnce(() -> wrist.setPosition(Constants.WRIST.CRADLE)),
  // new WaitUntilCommand(() -> wrist.atPosition()),
  // runOnce(() -> elevator.setPosition(e_goal)),
  // new WaitUntilCommand(() -> elevator.atPosition()),
  // runOnce(() -> wrist.setPosition(w_goal)),
  // new WaitUntilCommand(() -> wrist.atPosition()));

  @AutoLogOutput
  public Command shootAlgaeAtNetCmd(double e_goal, double w_goal) {
    return Commands.sequence(
        runOnce(() -> elevator.setPosition(e_goal)), runOnce(() -> wrist.setPosition(w_goal)));
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
