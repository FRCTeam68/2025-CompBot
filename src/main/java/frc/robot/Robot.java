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
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.util.PhoenixUtil;
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
  private Timer disabledTimer;
  private Timer autonTimer;
  private boolean prevEnableState;

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
    // TODO add this from sensor input
    Logger.recordMetadata("BatteryName", "ADD THIS IN FUTURE");
    Logger.recordMetadata("TuningMode", String.valueOf(Constants.tuningMode));

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

    // Elasic remote layout downloading
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // Rely on our custom alerts for disconnected controllers
    DriverStation.silenceJoystickConnectionWarning(true);

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    rioBus = new CANBus("rio");
    CANivoreBus = new CANBus("DRIVEbus");

    // uncomment the line below to log CTRE devices to usb stick
    SignalLogger.setPath("//media/sda1/logs");
    SignalLogger.start();
    // do not call the setPath and will be logged to rio at "/home/lvuser/logs"

    // Start AdvantageKit logger
    // This must be called after Instantiating the RobotContainer
    Logger.start();

    disabledTimer = new Timer();
    autonTimer = new Timer();
    prevEnableState = false;
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    PhoenixUtil.refreshAll();

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);

    robotContainer.updateAlerts();

    CANBusStatus canInfo = rioBus.getStatus();
    Logger.recordOutput("CANBUS/rio/Util", canInfo.BusUtilization);
    Logger.recordOutput("CANBUS/rio/Status", canInfo.Status.getName());
    if (!canInfo.Status.isOK())
      Logger.recordOutput("CANBUS/rio/Desc", canInfo.Status.getDescription());

    CANBusStatus canInfo2 = CANivoreBus.getStatus();
    Logger.recordOutput("CANBUS/DRIVEbus/Util", canInfo2.BusUtilization);
    Logger.recordOutput("CANBUS/DRIVEbus/Status", canInfo2.Status.getName());
    if (!canInfo2.Status.isOK())
      Logger.recordOutput("CANBUS/DRIVEbus/Desc", canInfo2.Status.getDescription());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    robotContainer.StopSubSystems();

    disabledTimer.reset(); // Reset the timer when entering disabled mode
    disabledTimer.start(); // Start the timer
    prevEnableState = false;
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    if (disabledTimer.advanceIfElapsed(1.0)) {
      // call less frequent than 20ms that periodic is called to reduce resource usage
      robotContainer.loadAutonPath();
      robotContainer.autonReadyStatus();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonTimer.reset();
    prevEnableState = false;

    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      if (Constants.currentMode.equals(Constants.Mode.SIM))
        robotContainer.setStartingMechanismPosition();
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    Logger.recordOutput("auton/autonTimer", autonTimer.get());

    if (prevEnableState == false && DriverStation.isEnabled()) {
      // going from disabled to enabled.  start timer
      autonTimer.start();
      prevEnableState = true;
    }

    if (autonTimer.hasElapsed(12.8)) {
      // tell drive to slow down incase its elevator is high at end of auton
      robotContainer.setNearEndOfAuton(true);
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    robotContainer.setAutonOn(false);
    robotContainer.setNearEndOfAuton(false);
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
  public void simulationPeriodic() {}
}
