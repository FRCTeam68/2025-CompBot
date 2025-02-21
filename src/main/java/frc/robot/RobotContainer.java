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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ElevatorWristSubSystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import frc.robot.subsystems.RangeSensorSubSystem;
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
  private final RollerSystem climber;
  private final RollerSystem intakeShooter;
  private final RangeSensorSubSystem intakeCoralSensor;
  private final ElevatorWristSubSystem elevatorWrist;
  private final LightsSubsystem lightsSubsystem;

  // Controller
  private final CommandXboxController m_xboxController = new CommandXboxController(0);
  private final CommandPS4Controller m_ps4Controller = new CommandPS4Controller(1);

  private final LoggedDashboardChooser<Command> autoChooser;

  private boolean m_pitModeActive = false;

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
        intakeShooter.setPieceCurrentThreshold(40);

        intakeCoralSensor =
            new RangeSensorSubSystem(
                "intakeCoral",
                Constants.INTAKE_CORAL_SENSOR.CANID,
                Constants.INTAKE_CORAL_SENSOR.CANBUS,
                Constants.INTAKE_CORAL_SENSOR.THRESHOLD);

        elevatorWrist = new ElevatorWristSubSystem();

        climber =
            new RollerSystem(
                "Climber",
                new RollerSystemIOTalonFX(
                    Constants.CLIMBER.CANID,
                    Constants.CLIMBER.CANBUS,
                    40,
                    false,
                    0,
                    false,
                    false,
                    1));
        // init tunables in the parent roller system
        climber.setPID(Constants.CLIMBER.SLOT0_CONFIGS);
        climber.setMotionMagic(Constants.CLIMBER.MOTIONMAGIC_CONFIGS);
        climber.setAtSetpointBand(.3);
        climber.setPieceCurrentThreshold(
            40); // does not have a piece but might want to use to detect overrun limits?

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

        intakeShooter =
            new RollerSystem(
                "IntakeShooter", new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 4, .1));
        // TBD, this needs an actual simulated sensor.....
        intakeCoralSensor =
            new RangeSensorSubSystem(
                "intakeCoral",
                Constants.INTAKE_CORAL_SENSOR.CANID,
                Constants.INTAKE_CORAL_SENSOR.CANBUS,
                Constants.INTAKE_CORAL_SENSOR.THRESHOLD);

        // TBD, this needs an actual simulated sensor.....
        elevatorWrist = new ElevatorWristSubSystem();

        climber =
            new RollerSystem("Climber", new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 4, .1));

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

        intakeShooter = new RollerSystem("IntakeShooter", new RollerSystemIO() {});
        intakeCoralSensor =
            new RangeSensorSubSystem(
                "intakeCoral",
                Constants.INTAKE_CORAL_SENSOR.CANID,
                Constants.INTAKE_CORAL_SENSOR.CANBUS,
                Constants.INTAKE_CORAL_SENSOR.THRESHOLD); // TBD, need better dummy

        // TBD, this needs an actual simulated sensor.....
        elevatorWrist = new ElevatorWristSubSystem();

        climber = new RollerSystem("Climber", new RollerSystemIO() {});

        // TBD, this needs an actual simulated sensor.....
        lightsSubsystem = new LightsSubsystem();
        break;
    }

    NamedCommands.registerCommand(
        "shoot", ManipulatorCommands.shootCoralCmd(intakeShooter, intakeCoralSensor));
    NamedCommands.registerCommand(
        "intake", ManipulatorCommands.intakeCoralCmd(intakeShooter, intakeCoralSensor));
    NamedCommands.registerCommand(
        "shootAlgaeAtP1", ManipulatorCommands.shootAlgaeP1Cmd(intakeShooter));
    NamedCommands.registerCommand(
        "intakeAlgae", ManipulatorCommands.intakeAlgaeA1A2Cmd(intakeShooter));
    NamedCommands.registerCommand("coralToL4", ManipulatorCommands.CoralL4Cmd(elevatorWrist));
    NamedCommands.registerCommand("coralToL3", ManipulatorCommands.CoralL3Cmd(elevatorWrist));
    NamedCommands.registerCommand("coralToL2", ManipulatorCommands.CoralL2Cmd(elevatorWrist));
    NamedCommands.registerCommand("coralToL1", ManipulatorCommands.CoralL1Cmd(elevatorWrist));
    NamedCommands.registerCommand("AlgaeAtA2", ManipulatorCommands.AlgaeAtA2(elevatorWrist));
    NamedCommands.registerCommand("AlgaeAtA1", ManipulatorCommands.AlgaeAtA1(elevatorWrist));
    NamedCommands.registerCommand("AlgaeToP1", ManipulatorCommands.AlgaeToP1(elevatorWrist));
    NamedCommands.registerCommand(
        "AlgaeToPreNet", ManipulatorCommands.AlgaeToPreNetCmd(elevatorWrist));
    NamedCommands.registerCommand(
        "shootAlgaeAtNet", ManipulatorCommands.ShootAlgaeToNetCmd(elevatorWrist, intakeShooter));

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

    // LEDSegment.Do1to4.setBandAnimation(LightsSubsystem.blue, 2);
    // LEDSegment.Do5to8.setColor(LightsSubsystem.orange);
    // LEDSegment.side1.setColor(LightsSubsystem.white);
    // LEDSegment.side1target.setColor(LightsSubsystem.white);
    // LEDSegment.side1heading.setColor(LightsSubsystem.white);
    // LEDSegment.side1distance.setColor(LightsSubsystem.white);

    SmartDashboard.putBoolean("laserCanTrip", false);
    SmartDashboard.putString("atPosition", "--");
    SmartDashboard.putBoolean("CLIMB", false);

    LEDSegment.all.setRainbowAnimation(4);
    // LEDSegment.leftside.setColor(LightsSubsystem.red);
    // LEDSegment.middle.setFadeAnimation(LightsSubsystem.orange, 1);
    // LEDSegment.rightside.setRainbowAnimation(4);
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

    m_xboxController
        .leftTrigger()
        .onTrue(ManipulatorCommands.intakeCoralCmd(intakeShooter, intakeCoralSensor));

    m_xboxController
        .rightTrigger()
        .onTrue(ManipulatorCommands.shootCoralCmd(intakeShooter, intakeCoralSensor));

    m_xboxController.leftBumper().onTrue(ManipulatorCommands.intakeAlgaeA1A2Cmd(intakeShooter));

    m_xboxController.rightBumper().onTrue(ManipulatorCommands.shootAlgaeP1Cmd(intakeShooter));

    m_xboxController.start().onTrue(Commands.runOnce(() -> intakeShooter.setSpeed(0)));

    m_ps4Controller.triangle().onTrue(ManipulatorCommands.CoralL4Cmd(elevatorWrist));

    m_ps4Controller.circle().onTrue(ManipulatorCommands.CoralL3Cmd(elevatorWrist));

    m_ps4Controller.square().onTrue(ManipulatorCommands.CoralL2Cmd(elevatorWrist));

    m_ps4Controller.cross().onTrue(ManipulatorCommands.CoralL1Cmd(elevatorWrist));

    m_ps4Controller.L1().onTrue(ManipulatorCommands.AlgaeToPreNetCmd(elevatorWrist));

    m_ps4Controller.L2().onTrue(ManipulatorCommands.AlgaeToP1(elevatorWrist));

    m_ps4Controller.R1().onTrue(ManipulatorCommands.AlgaeAtA2(elevatorWrist));
    m_ps4Controller.R2().onTrue(ManipulatorCommands.AlgaeAtA1(elevatorWrist));

    m_ps4Controller
        .options()
        .onTrue(
            Commands.runOnce(() -> elevatorWrist.zero())
                .andThen(() -> System.out.println("zero elevator and wrist")));

    m_ps4Controller
        .povUp()
        .onTrue(elevatorWrist.BumpElevatorPosition(Constants.ELEVATOR.BUMP_VALUE));

    m_ps4Controller
        .povDown()
        .onTrue(elevatorWrist.BumpElevatorPosition(-Constants.ELEVATOR.BUMP_VALUE));

    m_ps4Controller.povLeft().onTrue(elevatorWrist.BumpWristPosition(Constants.WRIST.BUMP_VALUE));

    m_ps4Controller.povRight().onTrue(elevatorWrist.BumpWristPosition(-Constants.WRIST.BUMP_VALUE));

    // //Left Joystick Y
    // m_ps4Controller.axisGreaterThan(1,0.7).whileTrue(Commands.run(()->m_NoteSubSystem.bumpIntake1Speed((-Constants.INTAKE.BUMP_VALUE))));
    // m_ps4Controller.axisLessThan(1,-0.7).whileTrue(Commands.run(()->m_NoteSubSystem.bumpIntake1Speed((Constants.INTAKE.BUMP_VALUE))));
    // //Right Joystick Y
    // m_ps4Controller.axisGreaterThan(5,0.7).whileTrue(Commands.run(()->m_NoteSubSystem.bumpIntake2Speed((-Constants.INTAKE.BUMP_VALUE))));
    // m_ps4Controller.axisLessThan(5,-0.7).whileTrue(Commands.run(()->m_NoteSubSystem.bumpIntake2Speed((Constants.INTAKE.BUMP_VALUE))));

    // m_ps4Controller.share().onTrue(Commands.runOnce(() -> m_NoteSubSystem.resetSetpoints()));

    m_ps4Controller
        .PS()
        .onTrue(
            Commands.runOnce(() -> m_pitModeActive = !m_pitModeActive)
                .andThen(() -> SmartDashboard.putBoolean("PitModeActive", m_pitModeActive)));

    // //Right Joystick Y
    // m_ps4Controller.axisGreaterThan(5,
    // 0.7).onTrue(ManipulatorCommands.DeployClimberCmd(climber));

    // m_ps4Controller.axisLessThan(5, -0.7).onTrue(ManipulatorCommands.RetractClimberCmd(climber));
    m_xboxController.leftBumper().onTrue(ManipulatorCommands.DeployClimberCmd(climber));

    m_xboxController.rightBumper().onTrue(ManipulatorCommands.RetractClimberCmd(climber));

    // //Left Joystick Y
    // m_ps4Controller.axisGreaterThan(1,0.7).whileTrue(
    // m_ps4Controller.axisLessThan(1,-0.7).whileTrue(
    climber.setDefaultCommand(
        Commands.run(() -> climber.setVolts(m_ps4Controller.getLeftY() * 12), climber));
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
