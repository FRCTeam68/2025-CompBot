package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteSubSystem;
import frc.robot.subsystems.NoteSubSystem.ActionRequest;
import org.littletonrobotics.junction.Logger;

public class IntakeNoteCmd extends Command {

  private NoteSubSystem m_noteSubSystem;
  private Timer cmdTimer;

  public IntakeNoteCmd(NoteSubSystem noteSubSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_noteSubSystem = noteSubSystem;
    addRequirements(m_noteSubSystem);
    cmdTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmdTimer.reset();
    cmdTimer.start();
    m_noteSubSystem.setAction(ActionRequest.INTAKENOTE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("IntakeCmd", "end");
  }

  // Command will run until the button is released, or autonomous timer hits
  @Override
  public boolean isFinished() {
    return (m_noteSubSystem.getHaveNote1());
  }
}
