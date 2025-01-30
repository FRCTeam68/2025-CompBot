package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.RollerSystem;
import org.littletonrobotics.junction.Logger;

public class IntakeNoteCmd extends Command {

  private RollerSystem intake;
  private Timer cmdTimer;

  public void IntakeCoralCmd(RollerSystem roller) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = roller;
    addRequirements(intake);
    cmdTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmdTimer.reset();
    cmdTimer.start();
    intake.setSpeed(Constants.INTAKE_SHOOTER.CORAL_INTAKE_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("IntakeCoralCmd", "end");
  }

  // Command will run until the button is released, or autonomous timer hits
  @Override
  public boolean isFinished() {
    return (false);
    // return (intake.getHaveCoral());

    // return ((m_noteSubSystem.atTargetAngle()) && (m_noteSubSystem.atTargetSpeed())
    // || (cmdTimer.hasElapsed(2)));
  }
}
