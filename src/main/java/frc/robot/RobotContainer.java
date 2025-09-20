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

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.LEDColor;
import frc.robot.Constants.LEDSegment;
import frc.robot.commands.*;
import frc.robot.commands.auton.Auton;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ShotVisualizer;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.LightsIO;
import frc.robot.subsystems.lights.LightsIOCANdle;
import frc.robot.subsystems.lights.LightsIOSim;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOSim;
import frc.robot.subsystems.rollers.RollerSystemIOTalonFX;
import frc.robot.subsystems.sensors.CoralSensor;
import frc.robot.subsystems.sensors.RangeSensorIO;
import frc.robot.subsystems.sensors.RangeSensorIOCANrange;
import frc.robot.subsystems.sensors.RangeSensorIOSim;
import frc.robot.subsystems.superstructure.ElevatorWristSubsystem;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.SuperstructureVisualizer;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.AllianceFlipUtil;
import lombok.Getter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private static Drive drive;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Climber climber;
  private final RollerSystem intakeShooter;
  private final RollerSystem intakeShooterLow;
  private final CoralSensor intakeCoralSensor;
  private final ElevatorWristSubsystem elevatorWrist;
  private final Lights LED;

  // Commands
  // (initalize commands to use @AutoLogOutput)
  // TODO Is there a better way to do this
  // TODO Possible use AutoLogOutputManager.addPackage("frc.lib");
  private ReefCentering reefCentering;
  private final Auton auton;

  @SuppressWarnings("unused")
  private final ManipulatorCommands manipulatorCommands = new ManipulatorCommands();

  // Controllers
  private static final CommandXboxController m_xboxController = new CommandXboxController(0);
  private static final CommandPS4Controller m_ps4Controller = new CommandPS4Controller(1);
  private static final Alert xboxDisconnectedAlert =
      new Alert("Xbox controller disconnected.", AlertType.kError);
  private static final Alert ps4DisconnectedAlert =
      new Alert("Ps4 controller disconnected.", AlertType.kError);

  // Autons
  //   private static String m_autonName;

  @Getter private static boolean m_overideMode = false;
  public static boolean m_autoshootOnPostDection = false;

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

        LED = new Lights(new LightsIOCANdle());

        vision =
            new Vision(
                LED,
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
            new CoralSensor(
                LED,
                new RangeSensorIOCANrange(
                    Constants.INTAKE_CORAL_SENSOR.ID, "rio", Constants.INTAKE_CORAL_SENSOR.CONFIG));

        climber = new Climber(new ClimberIOTalonFX());
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

        LED = new Lights(new LightsIOSim());

        vision = new Vision(LED, drive::addVisionMeasurement);

        intakeShooter =
            new RollerSystem(
                "IntakeShooter", new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 4, .1));

        intakeShooterLow =
            new RollerSystem(
                "IntakeShooterLow", new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 4, .1));

        intakeCoralSensor = new CoralSensor(LED, new RangeSensorIOSim("Coral"));

        climber = new Climber(new ClimberIOSim());
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

        LED = new Lights(new LightsIO() {});

        vision = new Vision(LED, drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        intakeShooter = new RollerSystem("IntakeShooter", new RollerSystemIO() {});
        intakeShooterLow = new RollerSystem("IntakeShooterLow", new RollerSystemIO() {});
        intakeCoralSensor = new CoralSensor(LED, new RangeSensorIO() {});

        climber = new Climber(new ClimberIO() {});
        break;
    }

    elevatorWrist = new ElevatorWristSubsystem(LED, drive::getPose);

    // Configure reef centering
    reefCentering = new ReefCentering(drive);

    // Configure auton
    auton = new Auton();

    // Warm up path following command
    FollowPathCommand.warmupCommand().schedule();

    // Configure the button bindings
    configureButtonBindings();

    // Configure visualizers
    SuperstructureVisualizer.setRobotPoseSupplier(drive::getPose);
    ShotVisualizer.setRobotPoseSupplier(drive::getPose);

    configureSysIdRoutines();

    configureTestingRoutines();

    SmartDashboard.putBoolean("Override Mode", m_overideMode);
    SmartDashboard.putBoolean("AutoShoot", false);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -m_xboxController.getLeftY(),
            () -> -m_xboxController.getLeftX(),
            () -> -m_xboxController.getRightX()));

    m_xboxController
        .a()
        .onTrue(
            Commands.runOnce(() -> elevatorWrist.setAutoShootOn(!elevatorWrist.isAutoShootOn()))
                .andThen(
                    () ->
                        SmartDashboard.putString(
                            "AutoShoot", elevatorWrist.isAutoShootOn() ? "ON" : "OFF")));

    // drive to nearest barge shotting location
    m_xboxController
        .b()
        .whileTrue(
            reefCentering
                .createPathCommand(ReefCentering.Side.Barge)
                .until(() -> reefCentering.haveConditionsChanged())
                .repeatedly());

    // lock to tag angle
    // m_xboxController
    //     .y()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -m_xboxController.getLeftY(),
    //             () -> -m_xboxController.getLeftX(),
    //             () -> vision.getTagPose(1).getRotation()));

    // // Reset gyro to 0° when B button is pressed
    // m_xboxController
    //     .back()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

    // Auto aim command example
    // @SuppressWarnings("resource")
    // PIDController aimController = new PIDController(0.2, 0.0, 0.0);
    // aimController.enableContinuousInput(-Math.PI, Math.PI);
    // m_xboxController
    //     .x()
    //     .whileTrue(
    //         Commands.startRun(
    //             () -> {
    //               aimController.reset();
    //             },
    //             () -> {
    //               DriveCommands.joystickDriveAtAngle(
    //                   drive,
    //                   () -> -m_xboxController.getLeftY(),
    //                   () -> -m_xboxController.getLeftX(),
    //                   () ->
    //                       new Rotation2d(
    //                           aimController.calculate(vision.getTargetX(1).getRadians())));
    //               Logger.recordOutput("DriveAtAngle/TargetX",
    //                vision.getTargetX(1).getRadians());
    //             },
    //             drive));

    m_xboxController
        .leftTrigger()
        .onTrue(
            ManipulatorCommands.intakeCmd(
                intakeShooter, intakeShooterLow, elevatorWrist, intakeCoralSensor, LED));

    m_xboxController
        .rightTrigger()
        .onTrue(ManipulatorCommands.shootCmd(intakeShooter, intakeShooterLow, elevatorWrist, LED));

    // Reef alignment
    m_xboxController.leftBumper().onTrue(new AlignToReefTagRelative(false, drive).withTimeout(3));
    m_xboxController.rightBumper().onTrue(new AlignToReefTagRelative(true, drive).withTimeout(3));

    // Reset robot rotation
    m_xboxController
        .back()
        .and(m_xboxController.x())
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                AllianceFlipUtil.apply(new Rotation2d(0)))))
                .andThen(() -> LED.setSolidColor(LEDColor.ORANGE, LEDSegment.ALL))
                .andThen(Commands.waitSeconds(1))
                .andThen(() -> LED.disableLEDs(LEDSegment.ALL)));

    // turn on megatag1 again to resync with april tags
    m_xboxController
        .back()
        .and(m_xboxController.y())
        .onTrue(
            Commands.runOnce(() -> vision.enableMegaTag1())
                .andThen(() -> LED.setBandAnimation(LEDColor.ORANGE, LEDSegment.ALL))
                .andThen(Commands.waitSeconds(2))
                .andThen(() -> LED.disableLEDs(LEDSegment.ALL)));

    m_xboxController
        .start()
        .onTrue(
            intakeShooter
                .setSpeedCmd(0)
                .andThen(intakeShooterLow.setSpeedCmd(0))
                .andThen(elevatorWrist.haltCmd())
                .andThen(Commands.runOnce(() -> System.out.printf("stop%n"))));

    m_xboxController
        .povUp()
        .whileTrue(
            reefCentering
                .createPathCommand(ReefCentering.Side.Middle)
                .until(() -> reefCentering.haveConditionsChanged())
                .repeatedly());
    m_xboxController
        .povDown()
        .whileTrue(
            reefCentering
                .createPathCommand(ReefCentering.Side.Back)
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
        .triangle()
        .onTrue(
            ManipulatorCommands.CoralL4Cmd(elevatorWrist)
                .andThen(Commands.waitSeconds(Constants.INTAKE_SHOOTER.CORAL_AUTO_SHOOT_DELAY))
                .andThen(
                    ManipulatorCommands.shootCmd(
                            intakeShooter, intakeShooterLow, elevatorWrist, LED)
                        .onlyIf(
                            () -> {
                              return elevatorWrist.isReefPostDetectedRaw()
                                  && elevatorWrist.isAutoShootOn()
                                  && ManipulatorCommands.isHavePiece();
                            })));

    m_ps4Controller
        .circle()
        .onTrue(
            ManipulatorCommands.CoralL3Cmd(elevatorWrist)
                .andThen(Commands.waitSeconds(Constants.INTAKE_SHOOTER.CORAL_AUTO_SHOOT_DELAY))
                .andThen(
                    ManipulatorCommands.shootCmd(
                            intakeShooter, intakeShooterLow, elevatorWrist, LED)
                        .onlyIf(
                            () -> {
                              return elevatorWrist.isReefPostDetectedRaw()
                                  && elevatorWrist.isAutoShootOn()
                                  && ManipulatorCommands.isHavePiece();
                            })));

    m_ps4Controller
        .square()
        .onTrue(
            ManipulatorCommands.CoralL2Cmd(elevatorWrist)
                .andThen(Commands.waitSeconds(Constants.INTAKE_SHOOTER.CORAL_AUTO_SHOOT_DELAY))
                .andThen(
                    ManipulatorCommands.shootCmd(
                            intakeShooter, intakeShooterLow, elevatorWrist, LED)
                        .onlyIf(
                            () -> {
                              return elevatorWrist.isReefPostDetectedRaw()
                                  && elevatorWrist.isAutoShootOn()
                                  && ManipulatorCommands.isHavePiece();
                            })));

    m_ps4Controller.cross().onTrue(ManipulatorCommands.CoralL1Cmd(elevatorWrist));

    m_ps4Controller.L1().onTrue(ManipulatorCommands.AlgaeToP1(elevatorWrist));
    m_ps4Controller.L2().onTrue(ManipulatorCommands.AlgaeToNetCmd(elevatorWrist));

    m_ps4Controller.R1().onTrue(ManipulatorCommands.AlgaeAtA1(elevatorWrist));
    m_ps4Controller.R2().onTrue(ManipulatorCommands.AlgaeAtA2(elevatorWrist));

    m_ps4Controller
        .povUp()
        .onTrue(
            Commands.either(
                ManipulatorCommands.BumpClimberCmd(climber, Constants.CLIMBER.BUMP_VALUE),
                elevatorWrist.BumpElevatorPosition(SuperstructureConstants.ELEVATOR.bump),
                () -> m_overideMode));

    m_ps4Controller
        .povDown()
        .onTrue(
            Commands.either(
                ManipulatorCommands.BumpClimberCmd(climber, -Constants.CLIMBER.BUMP_VALUE),
                elevatorWrist.BumpElevatorPosition(-SuperstructureConstants.ELEVATOR.bump),
                () -> m_overideMode));

    m_ps4Controller
        .share()
        .onTrue(
            Commands.runOnce(() -> m_overideMode = !m_overideMode)
                .andThen(() -> SmartDashboard.putBoolean("Override Mode", m_overideMode)));

    m_ps4Controller
        .povLeft()
        .onTrue(elevatorWrist.BumpWristPosition(SuperstructureConstants.WRIST.bump));

    m_ps4Controller
        .povRight()
        .onTrue(elevatorWrist.BumpWristPosition(-SuperstructureConstants.WRIST.bump));

    // use incase you notice red light on dashboard.
    // m_ps4Controller.share().onTrue(ManipulatorCommands.ZeroClimberCmd(climber));

    m_ps4Controller
        .options()
        .onTrue(
            ManipulatorCommands.CoralIntakePositionCmd(elevatorWrist)
                .andThen(
                    ManipulatorCommands.intakeCmd(
                        intakeShooter, intakeShooterLow, elevatorWrist, intakeCoralSensor, LED)));

    m_ps4Controller
        .touchpad()
        .onTrue(
            Commands.either(
                Commands.runOnce(() -> elevatorWrist.getElevator().zero()),
                Commands.none(),
                () -> {
                  return (elevatorWrist.getElevator().getPositionRotations() < 0.1)
                      && (elevatorWrist.getElevator().getSetpoint() == 0)
                      && (elevatorWrist.getElevator().getSpeed() == 0.1);
                }));

    // Right Joystick Y
    m_ps4Controller
        .axisGreaterThan(Axis.kRightY.value, 0.7)
        .onTrue(ManipulatorCommands.RetractClimberCmd(climber, LED));
    m_ps4Controller
        .axisLessThan(Axis.kRightY.value, -0.7)
        .onTrue(ManipulatorCommands.DeployClimberCmd(climber, LED));

    // Left Joystick Y
    m_ps4Controller
        .axisGreaterThan(Axis.kLeftY.value, 0.7)
        .onTrue(ManipulatorCommands.AlgaeCradle(elevatorWrist));
  }

  private void configureSysIdRoutines() {
    SmartDashboard.putData(
        "Drive/Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(drive));
    SmartDashboard.putData(
        "Drive/Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    SmartDashboard.putData(
        "Drive/Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData(
        "Drive/Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData(
        "Drive/Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData(
        "Drive/Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData(
        "Elevator/Elevator static",
        elevatorWrist.getElevator().staticElevatorCharacterization(2.0));
    SmartDashboard.putData(
        "Wrist/Wrist static", elevatorWrist.getWrist().staticWristCharacterization(2.0));
    SmartDashboard.putData("Climber/Climber static", climber.staticClimberCharacterization(2.0));
  }

  private void configureTestingRoutines() {
    SmartDashboard.putData(
        "Testing Routines/Functional Test",
        ManipulatorCommands.FunctionalTest(
            intakeShooter, intakeShooterLow, elevatorWrist, intakeCoralSensor, climber, LED));
    SmartDashboard.putData(
        "Testing Routines/Elevator Sequencing Test",
        ManipulatorCommands.TestElevatorWristSequencing(elevatorWrist));
    SmartDashboard.putData(
        "Testing Routines/Elevator Wrist to Zero Position",
        ManipulatorCommands.ElevatorWristZeroCmd(elevatorWrist));
    SmartDashboard.putData(
        "Testing Routines/Climber to Zero Position",
        ManipulatorCommands.climberToZeroCmd(climber, LED));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (auton.getAutoChooser().get() == null) {
      return null;
    }
    return auton.execute(
        drive, intakeShooter, intakeShooterLow, elevatorWrist, intakeCoralSensor, LED);
  }

  public void loadAutonPath() {
    auton.loadPath();
  }

  public void setStartingMechanismPosition() {
    elevatorWrist.getElevator().zero();
    elevatorWrist.getWrist().zero();
    intakeShooter.zero();
    intakeShooterLow.zero();
    climber.zero();
  }

  public void setAutonOn(boolean state) {
    elevatorWrist.setAutoShootOn(state);
    SmartDashboard.putBoolean("AutoShoot", state);
  }

  public void autonReadyStatus() {
    Pose2d curPose;
    double autonX = 0;
    double autonY = 0;
    double autonR = 0;
    double offsetX = 0;
    double offsetY = 0;
    double offsetR = 0;
    boolean offsetXOK = false;
    boolean offsetYOK = false;
    boolean offsetROK = false;
    boolean robotStateOK = false;

    // flip current pose if on the red side of the field
    // otherwise set as current pose
    curPose =
        (drive.getPose().getX() > (FieldConstants.fieldLength / 2))
            ? new Pose2d(
                new Translation2d(
                    FieldConstants.fieldLength - drive.getPose().getX(),
                    FieldConstants.fieldWidth - drive.getPose().getY()),
                drive.getPose().getRotation().rotateBy(Rotation2d.kPi))
            : drive.getPose();

    if (Math.abs(curPose.getY() - Constants.AutonStartPositions.middle.getY())
        < Math.abs(
            (Constants.AutonStartPositions.middle.getY()
                    - Constants.AutonStartPositions.right.getY())
                / 2)) {
      autonX = Constants.AutonStartPositions.middle.getX();
      autonY = Constants.AutonStartPositions.middle.getY();
      autonR = Constants.AutonStartPositions.middle.getRotation().getDegrees();
    } else if (curPose.getY() < Constants.AutonStartPositions.middle.getY()) {
      autonX = Constants.AutonStartPositions.right.getX();
      autonY = Constants.AutonStartPositions.right.getY();
      autonR = Constants.AutonStartPositions.right.getRotation().getDegrees();
    } else {
      autonX = Constants.AutonStartPositions.left.getX();
      autonY = Constants.AutonStartPositions.left.getY();
      autonR = Constants.AutonStartPositions.left.getRotation().getDegrees();
    }

    offsetX = curPose.getX() - autonX;
    offsetY = curPose.getY() - autonY;
    offsetR = Math.abs(curPose.getRotation().getDegrees()) - autonR;

    offsetXOK = (Math.abs(offsetX) < (Constants.AutonStartPositions.TRANSLATION_START_ERROR / 2));
    offsetYOK = (Math.abs(offsetY) < (Constants.AutonStartPositions.TRANSLATION_START_ERROR / 2));
    offsetROK = (Math.abs(offsetR) < (Constants.AutonStartPositions.ROTATION_START_ERROR / 2));

    SmartDashboard.putNumber("Auto/offset_X:", offsetX);
    SmartDashboard.putNumber("Auto/offset_Y:", offsetY);
    SmartDashboard.putNumber("Auto/offset_ROT:", offsetR);
    SmartDashboard.putBoolean("Auto/offset_X_OK:", offsetXOK);
    SmartDashboard.putBoolean("Auto/offset_Y_OK:", offsetYOK);
    SmartDashboard.putBoolean("Auto/offset_ROT_OK:", offsetROK);

    // Object m_autonName;
    //     try {
    //       m_autonName = auton.getAutoChooser().get().;
    // } catch (Exception e) {
    //   m_autonName = "";
    // }

    // robotStateOK =
    //     (m_autonName.contains("LEFT")
    //             || m_autonName.contains("CENTER")
    //             || m_autonName.contains("RIGHT"))
    //         && m_xboxController.isConnected()
    //         && m_ps4Controller.isConnected()
    //         && intakeCoralSensor.isDetected()
    //         && elevatorWrist.getWrist().getPosition() < 0.01
    //         && elevatorWrist.getElevator().getPositionRotations() < 0.01;

    // SmartDashboard.putBoolean(
    //     "Match Ready/auton_selected",
    //     m_autonName.contains("LEFT")
    //         || m_autonName.contains("CENTER")
    //         || m_autonName.contains("RIGHT"));
    SmartDashboard.putBoolean("Match Ready/coral_loaded", intakeCoralSensor.isDetected());
    SmartDashboard.putBoolean(
        "Match Ready/wrist_zeroed", elevatorWrist.getWrist().getPosition() < 0.01);
    SmartDashboard.putBoolean(
        "Match Ready/elevator_zeroed", elevatorWrist.getElevator().getPositionRotations() < 0.01);

    // set LEDs
    if (robotStateOK && offsetXOK && offsetYOK && offsetROK) {
      LED.setBandAnimation(LEDColor.GREEN, LEDSegment.ALL);
    } else {
      // all auton ready but position
      if (robotStateOK) {
        LED.setSolidColor(LEDColor.GREEN, LEDSegment.LEFT_SIDE);
        LED.setSolidColor(LEDColor.GREEN, LEDSegment.RIGHT_SIDE);
      } else {
        LED.setSolidColor(LEDColor.RED, LEDSegment.LEFT_SIDE);
        LED.setSolidColor(LEDColor.RED, LEDSegment.RIGHT_SIDE);
      }

      // X offset LEDs
      if (offsetXOK) {
        LED.setSolidColor(LEDColor.GREEN, LEDSegment.AUTON_X_LEFT);
        LED.setSolidColor(LEDColor.GREEN, LEDSegment.AUTON_X_RIGHT);
      } else {
        if (offsetX < 0) {
          LED.setSolidColor(LEDColor.RED, LEDSegment.AUTON_X_LEFT);
          LED.setSolidColor(LEDColor.GREEN, LEDSegment.AUTON_X_RIGHT);
        } else {
          LED.setSolidColor(LEDColor.GREEN, LEDSegment.AUTON_X_LEFT);
          LED.setSolidColor(LEDColor.RED, LEDSegment.AUTON_X_RIGHT);
        }
      }

      // Y offset LEDs
      if (offsetYOK) {
        LED.setSolidColor(LEDColor.GREEN, LEDSegment.AUTON_Y_LEFT);
        LED.setSolidColor(LEDColor.GREEN, LEDSegment.AUTON_Y_RIGHT);
      } else {
        if (offsetY < 0) {
          LED.setSolidColor(LEDColor.RED, LEDSegment.AUTON_Y_LEFT);
          LED.setSolidColor(LEDColor.GREEN, LEDSegment.AUTON_Y_RIGHT);
        } else {
          LED.setSolidColor(LEDColor.GREEN, LEDSegment.AUTON_Y_LEFT);
          LED.setSolidColor(LEDColor.RED, LEDSegment.AUTON_Y_RIGHT);
        }
      }

      // rotation offset LEDs
      if (offsetROK) {
        LED.setSolidColor(LEDColor.GREEN, LEDSegment.AUTON_R_LEFT);
        LED.setSolidColor(LEDColor.GREEN, LEDSegment.AUTON_R_RIGHT);
      } else {
        if (offsetR < 0) {
          LED.setSolidColor(LEDColor.RED, LEDSegment.AUTON_R_LEFT);
          LED.setSolidColor(LEDColor.GREEN, LEDSegment.AUTON_R_RIGHT);
        } else {
          LED.setSolidColor(LEDColor.GREEN, LEDSegment.AUTON_R_LEFT);
          LED.setSolidColor(LEDColor.RED, LEDSegment.AUTON_R_RIGHT);
        }
      }
    }
  }

  public void updateAlerts() {
    xboxDisconnectedAlert.set(!m_xboxController.isConnected());
    ps4DisconnectedAlert.set(!m_ps4Controller.isConnected());
  }

  public void StopSubSystems() {
    elevatorWrist.stop();
    intakeShooter.stop();
    intakeShooter.zero();
    intakeShooterLow.stop();
    climber.stop();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().clearComposedCommands();
  }
}
