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

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auton.autons;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  public static RobotContainer robotContainer;
  private CANBus rioBus;
  private CANBus CANivoreBus;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Check for valid swerve config
    var modules =
        new SwerveModuleConstants[] {
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight
        };
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    rioBus = new CANBus("rio");
    CANivoreBus = new CANBus("DRIVEbus");

    // initialize robot poses
    Logger.recordOutput(
        "RobotPose/Elevator Stage 1", new Pose3d[] {new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))});
    Logger.recordOutput(
        "RobotPose/Elevator Stage 2", new Pose3d[] {new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))});
    Logger.recordOutput(
        "RobotPose/Wrist", new Pose3d[] {new Pose3d(0.28575, 0, 0.411, new Rotation3d(0, 0, 0))});
    Logger.recordOutput(
        "RobotPose/Climber",
        new Pose3d[] {
          new Pose3d(
              -0.2921, 0, 0.4398003396, new Rotation3d(0, Units.degreesToRadians(47.559917), 0))
        });
    // use for robot model setup in AdvantageScope
    // Logger.recordOutput("RobotPose/Zero2d", new Pose2d[] {new Pose2d(0, 0, new Rotation2d(0,
    // 0))});
    // Logger.recordOutput(
    //    "RobotPose/Zero3d", new Pose3d[] {new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))});

    FollowPathCommand.warmupCommand().schedule();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);

    CANBusStatus canInfo = rioBus.getStatus();
    Logger.recordOutput("CANBUS/DRIVEbus/Util", canInfo.BusUtilization);
    Logger.recordOutput("CANBUS/DRIVEbus/Status", canInfo.Status.getName());
    if (!canInfo.Status.isOK())
      Logger.recordOutput("CANBUS/DRIVEbus/Desc", canInfo.Status.getDescription());

    CANBusStatus canInfo2 = CANivoreBus.getStatus();
    Logger.recordOutput("CANBUS/rio/Util", canInfo2.BusUtilization);
    Logger.recordOutput("CANBUS/rio/Status", canInfo2.Status.getName());
    if (!canInfo2.Status.isOK())
      Logger.recordOutput("CANBUS/rio/Desc", canInfo2.Status.getDescription());

    // led status lights
    if (canInfo.Status.isOK()) {
      LEDSegment.LED4.setColor(LightsSubsystem.green);
    } else {
      LEDSegment.LED4.setColor(LightsSubsystem.red);
    }
    if (canInfo2.Status.isOK()) {
      LEDSegment.LED5.setColor(LightsSubsystem.green);
    } else {
      LEDSegment.LED5.setColor(LightsSubsystem.red);
    }

    RobotContainer.logClimberPose();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    robotContainer.StopSubSystems();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    RobotContainer.loadAutonPath();
    RobotContainer.putAutonPoseToDashboard();
    RobotContainer.autonReadyStatus();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    RobotContainer.loadAutonPath();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
      autons.stopAllTimers();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    RobotContainer.loadAutonPath();
  }
}
