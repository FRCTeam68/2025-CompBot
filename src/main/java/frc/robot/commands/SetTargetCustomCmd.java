package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteSubSystem;
import org.littletonrobotics.junction.Logger;

public class SetTargetCustomCmd extends Command {

  private NoteSubSystem m_noteSubSystem;
  private double m_anglePosition;
  private double m_shooterSpeed;
  private Timer cmdTimer;

  public SetTargetCustomCmd(NoteSubSystem noteSubSystem, double angle, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_noteSubSystem = noteSubSystem;
    m_anglePosition = angle;
    m_shooterSpeed = speed;
    addRequirements(m_noteSubSystem);
    cmdTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmdTimer.reset();
    cmdTimer.start();
    m_noteSubSystem.setTargetCustom(m_anglePosition, m_shooterSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("TargetCustomCmd", "end");
  }

  // Command will run until the button is released, or autonomous timer hits
  @Override
  public boolean isFinished() {
    return ((m_noteSubSystem.atTargetAngle()) && (m_noteSubSystem.atTargetSpeed())
        || (cmdTimer.hasElapsed(2)));
  }
}
