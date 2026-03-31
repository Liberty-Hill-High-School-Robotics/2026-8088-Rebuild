package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class Eject extends Command {
  // The subsystem the command runs on
  private final Intake m_intake;

  public Eject(Intake subsystem) {
    m_intake = subsystem;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intake.eject();
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.intakeStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
