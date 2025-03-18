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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.subsystems.ReefCentering;
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
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private static Drive drive;
  private final Vision vision;
  private static RollerSystem climber;
  private final RollerSystem intakeShooter;
  private final RollerSystem intakeShooterLow;
  private static RangeSensorSubSystem intakeCoralSensor;
  private static ElevatorWristSubSystem elevatorWrist;
  private final LightsSubsystem lightsSubsystem;
  private ReefCentering reefCentering;

  public String selectedAutonName;

  // Controller
  private static final CommandXboxController m_xboxController = new CommandXboxController(0);
  private static final CommandPS4Controller m_ps4Controller = new CommandPS4Controller(1);

  private static LoggedDashboardChooser<Command> autoChooser;
  private static String m_autonName;
  private static boolean autonready;

  private boolean algaeCradleFlag = false;
  private boolean m_climberBump = false;

  private static boolean auton_start_position_ok = false;

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
                    0,
                    true,
                    1));
        // init tunables in the parent roller system
        intakeShooter.setPID(Constants.INTAKE_SHOOTER.SLOT0_CONFIGS);
        intakeShooter.setMotionMagic(Constants.INTAKE_SHOOTER.MOTIONMAGIC_CONFIGS);
        intakeShooter.setAtSetpointBand(.3);
        intakeShooter.setPieceCurrentThreshold(40);

        intakeShooterLow =
            new RollerSystem(
                "IntakeShooterLow",
                new RollerSystemIOTalonFX(
                    Constants.INTAKE_SHOOTER_LOW.CANID,
                    Constants.INTAKE_SHOOTER_LOW.CANBUS,
                    40,
                    true,
                    0,
                    false,
                    0,
                    false,
                    1));
        // init tunables in the parent roller system
        intakeShooterLow.setPID(Constants.INTAKE_SHOOTER_LOW.SLOT0_CONFIGS);
        intakeShooterLow.setMotionMagic(Constants.INTAKE_SHOOTER_LOW.MOTIONMAGIC_CONFIGS);
        intakeShooterLow.setAtSetpointBand(.3);
        intakeShooterLow.setPieceCurrentThreshold(40);

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
                    80,
                    true,
                    0,
                    false,
                    0,
                    true,
                    1));
        // init tunables in the parent roller system
        climber.setPID(Constants.CLIMBER.SLOT0_CONFIGS);
        climber.setMotionMagic(Constants.CLIMBER.MOTIONMAGIC_CONFIGS);
        climber.setAtSetpointBand(.3);
        climber.setPieceCurrentThreshold(
            40); // does not have a piece but might want to use to detect overrun limits?
        climber.zero();

        reefCentering = new ReefCentering(drive);

        lightsSubsystem = new LightsSubsystem();

        LEDSegment.LED6.setColor(LightsSubsystem.green);

        SmartDashboard.putString("BumpMode", "ELEVATOR");

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

        intakeShooterLow =
            new RollerSystem(
                "IntakeShooterLow", new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 4, .1));
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

        reefCentering = new ReefCentering(drive);

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
        intakeShooterLow = new RollerSystem("IntakeShooterLow", new RollerSystemIO() {});
        intakeCoralSensor =
            new RangeSensorSubSystem(
                "intakeCoral",
                Constants.INTAKE_CORAL_SENSOR.CANID,
                Constants.INTAKE_CORAL_SENSOR.CANBUS,
                Constants.INTAKE_CORAL_SENSOR.THRESHOLD); // TBD, need better dummy

        // TBD, this needs an actual simulated sensor.....
        elevatorWrist = new ElevatorWristSubSystem();

        climber = new RollerSystem("Climber", new RollerSystemIO() {});

        reefCentering = new ReefCentering(drive);

        // TBD, this needs an actual simulated sensor.....
        lightsSubsystem = new LightsSubsystem();
        break;
    }

    NamedCommands.registerCommand(
        "shoot", ManipulatorCommands.shootCmd(intakeShooter, intakeShooterLow, elevatorWrist));
    NamedCommands.registerCommand(
        "intake",
        ManipulatorCommands.intakeCmd(
            intakeShooter, intakeShooterLow, elevatorWrist, intakeCoralSensor));
    NamedCommands.registerCommand(
        "coralToL4", ManipulatorCommands.CoralL4Cmd(intakeShooterLow, elevatorWrist));
    NamedCommands.registerCommand(
        "coralToL3", ManipulatorCommands.CoralL3Cmd(intakeShooterLow, elevatorWrist));
    NamedCommands.registerCommand(
        "coralToL2", ManipulatorCommands.CoralL2Cmd(intakeShooterLow, elevatorWrist));
    NamedCommands.registerCommand(
        "toIntakeCoral",
        ManipulatorCommands.CoralIntakePositionCmd(intakeShooterLow, elevatorWrist));
    NamedCommands.registerCommand(
        "coralToL1", ManipulatorCommands.CoralL1Cmd(intakeShooterLow, elevatorWrist));
    NamedCommands.registerCommand(
        "algaeFromA2",
        ManipulatorCommands.AlgaeAtA2(intakeShooterLow, elevatorWrist, algaeCradleFlag));
    NamedCommands.registerCommand(
        "algaeFromA1",
        ManipulatorCommands.AlgaeAtA1(intakeShooterLow, elevatorWrist, algaeCradleFlag));
    NamedCommands.registerCommand(
        "algaeToP1",
        ManipulatorCommands.AlgaeToP1(intakeShooterLow, elevatorWrist, algaeCradleFlag));
    NamedCommands.registerCommand(
        "algaeToPreNet",
        ManipulatorCommands.AlgaeToNetCmd(intakeShooterLow, elevatorWrist, algaeCradleFlag));
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto/Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption(
        "Functional Test",
        ManipulatorCommands.FunctionalTest(
            intakeShooter, intakeShooterLow, elevatorWrist, intakeCoralSensor, climber));
    autoChooser.addOption(
        "Elevator Sequencing Test",
        ManipulatorCommands.TestElevatorWristSequencing(intakeShooterLow, elevatorWrist));
    // Set up SysId routines
    if (Constants.tuningMode) {
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
      autoChooser.addOption("Elevator static", elevatorWrist.staticElevatorCharacterization(2.0));
      autoChooser.addOption("Wrist static", elevatorWrist.staticWristCharacterization(2.0));
      autoChooser.addOption("Climber static", staticClimberCharacterization(2.0));
    }
    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putBoolean("HaveCoral", false);
    SmartDashboard.putString("atPosition", "--");
    SmartDashboard.putBoolean("CLIMB", false);

    // LEDSegment.all.setRainbowAnimation(2);
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
                () -> new Rotation2d(Units.degreesToRadians(0))));

    // Lock to 120° when A button is held
    m_xboxController
        .b()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -m_xboxController.getLeftY(),
                () -> -m_xboxController.getLeftX(),
                () -> Rotation2d.fromDegrees(120)));

    // lock to tag angle
    m_xboxController
        .y()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -m_xboxController.getLeftY(),
                () -> -m_xboxController.getLeftX(),
                () -> vision.getTagPose(1).getRotation()));

    // // Reset gyro to 0° when B button is pressed
    m_xboxController
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Auto aim command example
    @SuppressWarnings("resource")
    PIDController aimController = new PIDController(0.2, 0.0, 0.0);
    aimController.enableContinuousInput(-Math.PI, Math.PI);
    m_xboxController
        .x()
        .whileTrue(
            Commands.startRun(
                () -> {
                  aimController.reset();
                },
                () -> {
                  DriveCommands.joystickDriveAtAngle(
                      drive,
                      () -> -m_xboxController.getLeftY(),
                      () -> -m_xboxController.getLeftX(),
                      () ->
                          new Rotation2d(
                              aimController.calculate(vision.getTargetX(1).getRadians())));
                  Logger.recordOutput("DriveAtAngle/TargetX", vision.getTargetX(1).getRadians());
                },
                drive));

    m_xboxController
        .leftTrigger()
        .onTrue(
            ManipulatorCommands.intakeCmd(
                intakeShooter, intakeShooterLow, elevatorWrist, intakeCoralSensor));

    m_xboxController
        .rightTrigger()
        .onTrue(ManipulatorCommands.shootCmd(intakeShooter, intakeShooterLow, elevatorWrist));

    // Reef alignment
    m_xboxController.leftBumper().onTrue(new AlignToReefTagRelative(false, drive).withTimeout(3));
    m_xboxController.rightBumper().onTrue(new AlignToReefTagRelative(true, drive).withTimeout(3));

    m_xboxController
        .start()
        .onTrue(Commands.runOnce(() -> intakeShooter.setSpeed(0)).andThen(elevatorWrist.haltCmd()));

    m_ps4Controller
        .triangle()
        .onTrue(ManipulatorCommands.CoralL4Cmd(intakeShooterLow, elevatorWrist));

    m_ps4Controller
        .circle()
        .onTrue(ManipulatorCommands.CoralL3Cmd(intakeShooterLow, elevatorWrist));

    m_ps4Controller
        .square()
        .onTrue(ManipulatorCommands.CoralL2Cmd(intakeShooterLow, elevatorWrist));

    m_ps4Controller
        .cross()
        .onTrue(ManipulatorCommands.CoralIntakePositionCmd(intakeShooterLow, elevatorWrist));

    m_ps4Controller
        .L1()
        .onTrue(ManipulatorCommands.AlgaeToP1(intakeShooterLow, elevatorWrist, algaeCradleFlag));
    m_ps4Controller
        .L2()
        .onTrue(
            ManipulatorCommands.AlgaeToNetCmd(intakeShooterLow, elevatorWrist, algaeCradleFlag));

    m_ps4Controller
        .R1()
        .onTrue(ManipulatorCommands.AlgaeAtA1(intakeShooterLow, elevatorWrist, algaeCradleFlag));
    m_ps4Controller
        .R2()
        .onTrue(ManipulatorCommands.AlgaeAtA2(intakeShooterLow, elevatorWrist, algaeCradleFlag));

    m_ps4Controller.options().onTrue(Commands.runOnce(() -> putAutonPoseToDashboard()));

    m_xboxController
        .povDown()
        .whileTrue(
            reefCentering
                .createPathCommand(ReefCentering.Side.Back)
                .until(() -> reefCentering.haveConditionsChanged())
                .repeatedly());
    m_xboxController
        .povUp()
        .whileTrue(
            reefCentering
                .createPathCommand(ReefCentering.Side.Middle)
                .until(() -> reefCentering.haveConditionsChanged())
                .repeatedly());
    m_xboxController
        .povLeft()
        .whileTrue(
            reefCentering
                .createPathCommand(ReefCentering.Side.Left)
                .until(() -> reefCentering.haveConditionsChanged())
                .repeatedly());
    m_xboxController
        .povRight()
        .whileTrue(
            reefCentering
                .createPathCommand(ReefCentering.Side.Right)
                .until(() -> reefCentering.haveConditionsChanged())
                .repeatedly());

    m_ps4Controller
        .povUp()
        .onTrue(
            Commands.either(
                ManipulatorCommands.BumpClimberCmd(Constants.CLIMBER.BUMP_VALUE, climber),
                elevatorWrist.BumpElevatorPosition(Constants.ELEVATOR.BUMP_VALUE),
                () -> m_climberBump));

    m_ps4Controller
        .povDown()
        .onTrue(
            Commands.either(
                ManipulatorCommands.BumpClimberCmd(-Constants.CLIMBER.BUMP_VALUE, climber),
                elevatorWrist.BumpElevatorPosition(-Constants.ELEVATOR.BUMP_VALUE),
                () -> m_climberBump));

    m_ps4Controller
        .share()
        .onTrue(
            Commands.runOnce(() -> m_climberBump = !m_climberBump)
                .andThen(
                    () ->
                        SmartDashboard.putString(
                            "BumpMode", m_climberBump ? "CLIMBER" : "ELEVATOR")));

    m_ps4Controller.povLeft().onTrue(elevatorWrist.BumpWristPosition(Constants.WRIST.BUMP_VALUE));

    m_ps4Controller.povRight().onTrue(elevatorWrist.BumpWristPosition(-Constants.WRIST.BUMP_VALUE));

    // //Left Joystick Y
    // m_ps4Controller.axisGreaterThan(1,0.7).whileTrue(Commands.run(()->m_NoteSubSystem.bumpIntake1Speed((-Constants.INTAKE.BUMP_VALUE))));
    // m_ps4Controller.axisLessThan(1,-0.7).whileTrue(Commands.run(()->m_NoteSubSystem.bumpIntake1Speed((Constants.INTAKE.BUMP_VALUE))));
    // //Right Joystick Y
    // m_ps4Controller.axisGreaterThan(5,0.7).whileTrue(Commands.run(()->m_NoteSubSystem.bumpIntake2Speed((-Constants.INTAKE.BUMP_VALUE))));
    // m_ps4Controller.axisLessThan(5,-0.7).whileTrue(Commands.run(()->m_NoteSubSystem.bumpIntake2Speed((Constants.INTAKE.BUMP_VALUE))));

    // use incase you notice red light on dashboard.
    // m_ps4Controller.share().onTrue(ManipulatorCommands.ZeroClimberCmd(climber));

    // m_ps4Controller
    //     .options()
    //     .onTrue(ManipulatorCommands.TestElevatorWristSequencing(elevatorWrist));

    m_ps4Controller.PS().onTrue(ManipulatorCommands.CoralL1Cmd(intakeShooterLow, elevatorWrist));

    // m_ps4Controller
    //     .touchpad()
    //     .onTrue(ManipulatorCommands.ElevatorWristZeroCmd(intakeShooterLow, elevatorWrist));

    // Right Joystick Y
    m_ps4Controller
        .axisGreaterThan(Axis.kRightY.value, 0.7)
        .onTrue(ManipulatorCommands.RetractClimberCmd(climber));
    m_ps4Controller
        .axisLessThan(Axis.kRightY.value, -0.7)
        .onTrue(ManipulatorCommands.DeployClimberCmd(climber));

    // Left Joystick Y
    m_ps4Controller
        .axisGreaterThan(Axis.kLeftY.value, 0.7)
        .onTrue(ManipulatorCommands.AlgaeCradle(intakeShooterLow, elevatorWrist))
        .onTrue(Commands.runOnce(() -> algaeCradleFlag = true))
        .onFalse(Commands.runOnce(() -> algaeCradleFlag = false));

    // climber.setDefaultCommand(
    //     Commands.run(() -> climber.setVolts(-m_ps4Controller.getLeftY() * 12), climber)
    //         .unless(
    //             () -> {
    //               return Math.abs(m_ps4Controller.getLeftY()) < .05;
    //             }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public static void putAutonPoseToDashboard() {
    Pose2d curPose = AllianceFlipUtil.apply(drive.getPose());
    double autonX = 0;
    double autonY = 0;
    double autonR = 0;
    double offsetX = 0;
    double offsetY = 0;
    double offsetR = 0;
    boolean offsetXOK = false;
    boolean offsetYOK = false;
    boolean offsetROK = false;

    m_autonName = autoChooser.get().getName();

    if (m_autonName.contains("LEFT")) {
      autonX = Constants.AutonStartPositions.left.getX();
      autonY = Constants.AutonStartPositions.left.getY();
      autonR = Constants.AutonStartPositions.left.getRotation().getDegrees();
    } else if (m_autonName.contains("CENTER")) {
      autonX = Constants.AutonStartPositions.middle.getX();
      autonY = Constants.AutonStartPositions.middle.getY();
      autonR = Constants.AutonStartPositions.middle.getRotation().getDegrees();
    } else if (m_autonName.contains("RIGHT")) {
      autonX = Constants.AutonStartPositions.right.getX();
      autonY = Constants.AutonStartPositions.right.getY();
      autonR = Constants.AutonStartPositions.right.getRotation().getDegrees();
    }

    offsetX = curPose.getX() - autonX;
    offsetY = curPose.getY() - autonY;
    offsetR = Math.abs(curPose.getRotation().getDegrees()) - autonR;

    offsetXOK =
        (Math.abs(offsetX) < (Constants.AutonStartPositions.TRANSLATION_START_ERROR / 2))
            ? true
            : false;
    offsetYOK =
        (Math.abs(offsetY) < (Constants.AutonStartPositions.TRANSLATION_START_ERROR / 2))
            ? true
            : false;
    offsetROK =
        (Math.abs(offsetR) < (Constants.AutonStartPositions.ROTATION_START_ERROR / 2))
            ? true
            : false;

    SmartDashboard.putNumber("Auto/offset_X:", offsetX);
    SmartDashboard.putNumber("Auto/offset_Y:", offsetY);
    SmartDashboard.putNumber("Auto/offset_ROT:", offsetR);
    SmartDashboard.putBoolean("Auto/offset_X_OK:", offsetXOK);
    SmartDashboard.putBoolean("Auto/offset_Y_OK:", offsetYOK);
    SmartDashboard.putBoolean("Auto/offset_ROT_OK:", offsetROK);

    if (offsetXOK && offsetYOK && offsetROK) {
      auton_start_position_ok = true;
    } else {
      auton_start_position_ok = false;
      if (offsetXOK) {
        LEDSegment.autonXLeft.setColor(LightsSubsystem.green);
        LEDSegment.autonXRight.setColor(LightsSubsystem.green);
      } else {
        if (offsetX < 0) {
          LEDSegment.autonXLeft.setColor(LightsSubsystem.red);
          LEDSegment.autonXRight.setColor(LightsSubsystem.green);
        } else {
          LEDSegment.autonXLeft.setColor(LightsSubsystem.green);
          LEDSegment.autonXRight.setColor(LightsSubsystem.red);
        }
      }
      if (offsetYOK) {
        LEDSegment.autonYLeft.setColor(LightsSubsystem.green);
        LEDSegment.autonYRight.setColor(LightsSubsystem.green);
      } else {
        if (offsetY < 0) {
          LEDSegment.autonYLeft.setColor(LightsSubsystem.red);
          LEDSegment.autonYRight.setColor(LightsSubsystem.green);
        } else {
          LEDSegment.autonYLeft.setColor(LightsSubsystem.green);
          LEDSegment.autonYRight.setColor(LightsSubsystem.red);
        }
      }
      if (offsetROK) {
        LEDSegment.autonRLeft.setColor(LightsSubsystem.green);
        LEDSegment.autonRRight.setColor(LightsSubsystem.green);
      } else {
        if (offsetR < 0) {
          LEDSegment.autonRLeft.setColor(LightsSubsystem.red);
          LEDSegment.autonRRight.setColor(LightsSubsystem.green);
        } else {
          LEDSegment.autonRLeft.setColor(LightsSubsystem.green);
          LEDSegment.autonRRight.setColor(LightsSubsystem.red);
        }
      }
    }
  }

  public static void autonReadyStatus() {
    autonready =
        (m_autonName.contains("LEFT")
                || m_autonName.contains("CENTER")
                || m_autonName.contains("RIGHT"))
            && m_xboxController.isConnected()
            && m_ps4Controller.isConnected()
            && intakeCoralSensor.havePiece()
            && elevatorWrist.getWrist().getPosition() < 0.01
            && elevatorWrist.getElevator().getPosition() < 0.01;
    if (autonready && auton_start_position_ok) {
      LEDSegment.all.setBandAnimation(LightsSubsystem.green, 4);
    } else if (autonready) {
      LEDSegment.leftside.setColor(LightsSubsystem.green);
      LEDSegment.rightside.setColor(LightsSubsystem.green);
    } else {
      LEDSegment.leftside.setColor(LightsSubsystem.red);
      LEDSegment.rightside.setColor(LightsSubsystem.red);
    }

    SmartDashboard.putBoolean(
        "Match Ready/auton_selected",
        m_autonName.contains("LEFT")
            || m_autonName.contains("CENTER")
            || m_autonName.contains("RIGHT"));
    SmartDashboard.putBoolean("Match Ready/xbox_connected", m_xboxController.isConnected());
    SmartDashboard.putBoolean("Match Ready/ps4_connected", m_ps4Controller.isConnected());
    SmartDashboard.putBoolean("Match Ready/coral_loaded", intakeCoralSensor.havePiece());
    SmartDashboard.putBoolean(
        "Match Ready/wrist_zeroed", elevatorWrist.getWrist().getPosition() < 0.01);
    SmartDashboard.putBoolean(
        "Match Ready/elevator_zeroed", elevatorWrist.getElevator().getPosition() < 0.01);
  }

  public static void logClimberPose() {
    Logger.recordOutput(
        "RobotPose/Climber",
        new Pose3d[] {
          new Pose3d(
              -0.2921,
              0,
              0.4398003396,
              new Rotation3d(
                  0,
                  Units.degreesToRadians(47.559917 - (climber.getPosition() / 125 * (12 / 32))),
                  0))
        });
  }

  public Command staticClimberCharacterization(double outputRampRate) {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              // stopProfile = true;
              timer.restart();
            },
            () -> {
              state.characterizationOutput = outputRampRate * timer.get();
              climber.setVolts(state.characterizationOutput);
              Logger.recordOutput(
                  "Climber/StaticCharacterizationOutput", state.characterizationOutput);
            })
        .until(() -> climber.getPosition() >= Constants.CLIMBER.DEPLOY)
        .finallyDo(
            () -> {
              // stopProfile = false;
              timer.stop();
              Logger.recordOutput("Climber/CharacterizationOutput", state.characterizationOutput);
            });
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }

  public void StopSubSystems() {
    elevatorWrist.stop();
    intakeShooter.stop();
    intakeShooterLow.stop();
    climber.stop();
  }
}
