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

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LaserCanSystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOSim;
import frc.robot.subsystems.rollers.RollerSystemIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  //   private final RollerSystem climber;
  private final RollerSystem intakeShooter;
  private final LaserCanSystem intakeCoralSensor;
  private final RollerSystem wrist;
  private final RollerSystem elevator;
  private final RollerSystem elevatorFollower;
  private final LightsSubsystem lightsSubsystem;

  // Controller
  private final CommandXboxController m_xboxController = new CommandXboxController(0);
  private final CommandPS4Controller m_ps4Controller = new CommandPS4Controller(1);

  DigitalInput m_noteSensor2 = new DigitalInput(0);
  Trigger m_NoteSensorTrigger2 = new Trigger(m_noteSensor2::get);

  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));

        // climber =
        //     new RollerSystem(
        //         "Climber",
        //         new RollerSystemIOTalonFX(
        //             Constants.CLIMBER.CANID,
        //             Constants.CLIMBER.CANBUS,
        //             40,
        //             false,
        //             0,
        //             false,
        //             false,
        //             1,
        //             Constants.CLIMBER.SLOT0_CONFIGS));
        // climber.setPID(Constants.CLIMBER.SLOT0_CONFIGS); // init tunables in the parent roller
        // system
        // climber.setMotionMagic(Constants.CLIMBER.MOTIONMAGIC_CONFIGS);
        // climber.setAtSetpointBand(.3);

        intakeShooter =
            new RollerSystem(
                "IntakeShooter",
                new RollerSystemIOTalonFX(
                    Constants.INTAKE_SHOOTER.CANID,
                    Constants.INTAKE_SHOOTER.CANBUS,
                    40,
                    false,
                    0,
                    false,
                    false,
                    1));
        // init tunables in the parent roller system
        intakeShooter.setPID(Constants.INTAKE_SHOOTER.SLOT0_CONFIGS);
        intakeShooter.setMotionMagic(Constants.INTAKE_SHOOTER.MOTIONMAGIC_CONFIGS);
        intakeShooter.setAtSetpointBand(.3);

        intakeCoralSensor =
            new LaserCanSystem(
                "intakeCoral",
                Constants.INTAKE_CORAL_SENSOR.CANID,
                Constants.INTAKE_CORAL_SENSOR.THRESHOLD);

        wrist =
            new RollerSystem(
                "Wrist",
                new RollerSystemIOTalonFX(
                    Constants.WRIST.CANID, Constants.WRIST.CANBUS, 40, false, 0, false, false, 1));
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
                    false,
                    1));
        // init tunables in the parent roller system
        elevatorFollower.setPID(Constants.ELEVATOR.SLOT0_CONFIGS);
        elevatorFollower.setMotionMagic(Constants.ELEVATOR.MOTIONMAGIC_CONFIGS);
        elevatorFollower.setAtSetpointBand(.3);

        // climber =
        //     new RollerSystem(
        //         "Climber",
        //         new RollerSystemIOTalonFX(
        //             Constants.CLIMBER.LEFT_CANID,
        //             Constants.CLIMBER.CANBUS,
        //             40,
        //             false,
        //             0,
        //             false,
        //             false,
        //             1));
        // // init tunables in the parent roller system
        // climber.setPID(Constants.CLIMBER.SLOT0_CONFIGS);
        // climber.setMotionMagic(Constants.CLIMBER.MOTIONMAGIC_CONFIGS);
        // climber.setAtSetpointBand(.3);

        lightsSubsystem = new LightsSubsystem();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        // (Use same number of dummy implementations as the real robot)
        // no sim for limelight????  use blank
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        // climber =
        //     new RollerSystem(
        //         "Climber", new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 4, .1));

        intakeShooter =
            new RollerSystem(
                "IntakeShooter", new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 4, .1));
        // TBD, this needs an actual simulated sensor.....
        intakeCoralSensor = new LaserCanSystem("intakeCoral", 37, 30);

        wrist = new RollerSystem("Wrist", new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 4, .1));
        elevator =
            new RollerSystem("Elevator", new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 4, .1));
        elevatorFollower =
            new RollerSystem(
                "ElevatorFollower", new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 4, .1));
        // climber =
        //     new RollerSystem(
        //         "Climber", new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 4, .1));

        // TBD, this needs an actual simulated sensor.....
        lightsSubsystem = new LightsSubsystem();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // (Use same number of dummy implementations as the real robot)
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        // climber = new RollerSystem("Climber", new RollerSystemIO() {});

        intakeShooter = new RollerSystem("IntakeShooter", new RollerSystemIO() {});
        intakeCoralSensor = new LaserCanSystem("intakeCoral", 37, 30); // TBD, need better dummy
        wrist = new RollerSystem("Wrist", new RollerSystemIO() {});
        elevator = new RollerSystem("Elevator", new RollerSystemIO() {});
        elevatorFollower = new RollerSystem("ElevatorFollower", new RollerSystemIO() {});
        // climber = new RollerSystem("Climber", new RollerSystemIO() {});

        // TBD, this needs an actual simulated sensor.....
        lightsSubsystem = new LightsSubsystem();
        break;
    }

    NamedCommands.registerCommand(
        "shoot",
        Commands.runOnce(() -> intakeShooter.setSpeed(Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED)));
    NamedCommands.registerCommand(
        "intake",
        Commands.runOnce(
            () -> intakeShooter.setSpeed(Constants.INTAKE_SHOOTER.CORAL_INTAKE_SPEED)));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    LEDSegment.Do1to4.setBandAnimation(LightsSubsystem.blue, 2);
    LEDSegment.Do5to8.setColor(LightsSubsystem.orange);
    LEDSegment.side1.setColor(LightsSubsystem.white);
    LEDSegment.side1target.setColor(LightsSubsystem.white);
    LEDSegment.side1heading.setColor(LightsSubsystem.white);
    LEDSegment.side1distance.setColor(LightsSubsystem.white);

    SmartDashboard.putBoolean("laserCanTrip", false);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    Trigger m_LaserCanTrigger = new Trigger(intakeCoralSensor::havePiece);
    m_LaserCanTrigger
        .onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("laserCanTrip", true)))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("laserCanTrip", false)));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -m_xboxController.getLeftY(),
            () -> -m_xboxController.getLeftX(),
            () -> -m_xboxController.getRightX()));

    // Lock to 0° when A button is held
    m_xboxController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -m_xboxController.getLeftY(),
                () -> -m_xboxController.getLeftX(),
                () -> new Rotation2d()));

    m_xboxController
        .y()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -m_xboxController.getLeftY(),
                () -> -m_xboxController.getLeftX(),
                () -> new Rotation2d(vision.getTargetX(0).getRadians())));

    // Switch to X pattern when X button is pressed
    m_xboxController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    m_xboxController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Auto aim command example
    // @SuppressWarnings("resource")
    // PIDController aimController = new PIDController(0.2, 0.0, 0.0);
    // aimController.enableContinuousInput(-Math.PI, Math.PI);
    // m_xboxController
    //     .y()
    //     .whileTrue(
    //         Commands.startRun(
    //             () -> {  aimController.reset(); },
    // 			() -> {  DriveCommands.joystickDriveAtAngle(
    //                 drive,
    //                 () -> -m_xboxController.getLeftY(),
    //                 () -> -m_xboxController.getLeftX(),
    //                 aimController.calculate(vision.getTargetX(0).getRadians()));
    //             },
    //             drive));

    m_xboxController
        .back()
        .onTrue(Commands.runOnce(() -> drive.setPose(new Pose2d()), drive).ignoringDisable(true));

    // m_xboxController
    //     .y()
    //     .onTrue(Commands.runOnce(() -> m_NoteSubSystem.setTarget(Target.SPEAKER_PODIUM)));
    // m_xboxController.b().onTrue(Commands.runOnce(() -> m_NoteSubSystem.setTarget(Target.AMP)));
    // m_xboxController.x().onTrue(Commands.runOnce(()->m_NoteSubSystem.setTarget(Target.TRAP)));
    // m_xboxController.a().onTrue(Commands.runOnce(() ->
    // m_NoteSubSystem.setTarget(Target.SPEAKER)));

    // for trial
    // m_xboxController.a().onTrue(new SetTargetCustomCmd(m_NoteSubSystem, Constants.ANGLE.SPEAKER,
    // Constants.SHOOTER.SPEAKER_SHOOT_SPEED));

    m_xboxController
        .leftTrigger()
        .onTrue(
            intakeShooter
                .setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_INTAKE_SPEED)
                .andThen(() -> LEDSegment.side1.setBandAnimation(LightsSubsystem.blue, .5))
                .andThen(new WaitUntilCommand(() -> intakeCoralSensor.havePiece()))
                .withTimeout(5)
                .finallyDo(() -> LEDSegment.side1.setColor(LightsSubsystem.blue))
                .handleInterrupt(() -> LEDSegment.side1.setColor(LightsSubsystem.white))
                .andThen(intakeShooter.setSpeedCmd(0)));
    m_xboxController
        .rightTrigger()
        .onTrue(
            intakeShooter
                .setSpeedCmd(Constants.INTAKE_SHOOTER.CORAL_SHOOT_SPEED)
                .andThen(() -> LEDSegment.side1.setColor(LightsSubsystem.red))
                .andThen(new WaitUntilCommand(() -> intakeCoralSensor.havePiece() == false))
                .withTimeout(2)
                .finallyDo(() -> LEDSegment.side1.setColor(LightsSubsystem.white))
                .handleInterrupt(() -> LEDSegment.side1.setFadeAnimation(LightsSubsystem.red, 0.5))
                .andThen(intakeShooter.setSpeedCmd(0)));
    // m_xboxController.leftBumper().onTrue(Commands.runOnce(() ->
    // m_NoteSubSystem.setAction(ActionRequest.SPIT_NOTE2)));
    // m_xboxController.rightBumper().onTrue(Commands.runOnce(() ->
    // m_NoteSubSystem.setAction(ActionRequest.SHOOT_SPINUP)));
    m_xboxController
        .start()
        .onTrue(
            Commands.runOnce(() -> intakeShooter.setSpeed(0))
                .andThen(Commands.runOnce(() -> wrist.setPosition(0))));

    m_ps4Controller
        .triangle()
        .onTrue(
            Commands.runOnce(() -> wrist.setPosition(Constants.WRIST.L4))
                .andThen(() -> LEDSegment.side1.setBandAnimation(LightsSubsystem.blue, .5))
                .andThen(new WaitUntilCommand(() -> wrist.getAtSetpoint()))
                .withTimeout(5)
                .andThen(() -> LEDSegment.side1.setColor(LightsSubsystem.white))
                .andThen(Commands.runOnce(() -> SmartDashboard.putString("Goal", "L4"))));
    // .andThen(Commands.runOnce(() -> elevator.setPosition(Constants.ELEVATOR.L4))));

    m_ps4Controller
        .circle()
        .onTrue(
            Commands.runOnce(() -> wrist.setPosition(Constants.WRIST.L3))
                .andThen(() -> LEDSegment.side1.setBandAnimation(LightsSubsystem.blue, .5))
                .andThen(new WaitUntilCommand(() -> wrist.getAtSetpoint()))
                .withTimeout(5)
                .andThen(() -> LEDSegment.side1.setColor(LightsSubsystem.white))
                .andThen(Commands.runOnce(() -> SmartDashboard.putString("Goal", "L3"))));
    // .andThen(Commands.runOnce(() -> elevator.setPosition(Constants.ELEVATOR.L3))));

    m_ps4Controller
        .square()
        .onTrue(
            Commands.runOnce(() -> wrist.setPosition(Constants.WRIST.L2))
                .andThen(() -> LEDSegment.side1.setBandAnimation(LightsSubsystem.blue, .5))
                .andThen(new WaitUntilCommand(() -> wrist.getAtSetpoint()))
                .withTimeout(5)
                .andThen(() -> LEDSegment.side1.setColor(LightsSubsystem.white))
                .andThen(Commands.runOnce(() -> SmartDashboard.putString("Goal", "L2"))));
    // .andThen(Commands.runOnce(() -> elevator.setPosition(Constants.ELEVATOR.L2))));

    m_ps4Controller
        .cross()
        .onTrue(
            Commands.runOnce(() -> wrist.setPosition(Constants.WRIST.L1))
                .andThen(() -> LEDSegment.side1.setBandAnimation(LightsSubsystem.blue, .5))
                .andThen(new WaitUntilCommand(() -> wrist.getAtSetpoint()))
                .withTimeout(5)
                .andThen(() -> LEDSegment.side1.setColor(LightsSubsystem.white))
                .andThen(Commands.runOnce(() -> SmartDashboard.putString("Goal", "L1"))));
    // .andThen(Commands.runOnce(() -> elevator.setPosition(Constants.ELEVATOR.L1))));

    // m_ps4Controller.L1().onTrue(Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.FEEDSTATION_SPIN)));
    // m_ps4Controller.L2().onTrue(Commands.runOnce(() ->
    // m_NoteSubSystem.setAction(ActionRequest.DISLODGE_WITH_SHOOTER)));
    m_ps4Controller.R1().onTrue(Commands.runOnce(() -> wrist.setPosition(Constants.WRIST.A2)));
    m_ps4Controller.R2().onTrue(Commands.runOnce(() -> wrist.setPosition(Constants.WRIST.A1)));

    // m_ps4Controller.touchpad().onTrue(Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.STOP)));
    // m_ps4Controller.options().onTrue(Commands.runOnce(() ->
    // m_NoteSubSystem.setHaveNote1(false)));

    // m_ps4Controller.povLeft()
    //     .onTrue(Commands.runOnce(() ->
    //                 intakeShooter.setSpeed(
    //                     intakeShooter.getSpeed() - Constants.INTAKE_SHOOTER.BUMP_VALUE)));
    // m_ps4Controller.povRight()
    //     .onTrue(Commands.runOnce(() ->
    //                 intakeShooter.setSpeed(
    //                     intakeShooter.getSpeed() + Constants.INTAKE_SHOOTER.BUMP_VALUE)));
    m_ps4Controller
        .povUp()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.setPosition(elevator.getPosition() + Constants.ELEVATOR.BUMP_VALUE)));
    // on 2024 bot, positive is intake side up, shooter side looks to be going down
    m_ps4Controller
        .povDown()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.setPosition(elevator.getPosition() - Constants.ELEVATOR.BUMP_VALUE)));
    m_ps4Controller
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () -> wrist.setPosition(wrist.getPosition() + Constants.WRIST.BUMP_VALUE)));
    m_ps4Controller
        .povRight()
        .onTrue(
            Commands.runOnce(
                () -> wrist.setPosition(wrist.getPosition() - Constants.WRIST.BUMP_VALUE)));

    // //Left Joystick Y
    // m_ps4Controller.axisGreaterThan(1,0.7).whileTrue(Commands.run(()->m_NoteSubSystem.bumpIntake1Speed((-Constants.INTAKE.BUMP_VALUE))));
    // m_ps4Controller.axisLessThan(1,-0.7).whileTrue(Commands.run(()->m_NoteSubSystem.bumpIntake1Speed((Constants.INTAKE.BUMP_VALUE))));
    // //Right Joystick Y
    // m_ps4Controller.axisGreaterThan(5,0.7).whileTrue(Commands.run(()->m_NoteSubSystem.bumpIntake2Speed((-Constants.INTAKE.BUMP_VALUE))));
    // m_ps4Controller.axisLessThan(5,-0.7).whileTrue(Commands.run(()->m_NoteSubSystem.bumpIntake2Speed((Constants.INTAKE.BUMP_VALUE))));

    // m_ps4Controller.share().onTrue(Commands.runOnce(() -> m_NoteSubSystem.resetSetpoints()));

    // m_ps4Controller.PS().onTrue(
    //         Commands.runOnce(() -> m_climbActive = !m_climbActive)
    //             .andThen(() -> m_Climber.setPitMode(m_climbActive))
    //             .andThen(() -> SmartDashboard.putBoolean("ClimberPitMode", m_climbActive)));

    // climber.setDefaultCommand(
    //     Commands.run(() -> climber.setVolts(m_ps4Controller.getLeftY() * 12), climber));

    m_NoteSensorTrigger2
        .onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("NoteSensor2", true)))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("NoteSensor2", false)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  //   public void StopSubSystems() {
  //     m_NoteSubSystem.setAction(ActionRequest.STOP_ALL);
  //   }
}
