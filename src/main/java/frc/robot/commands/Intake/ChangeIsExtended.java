package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class ChangeIsExtended extends Command {
  // The subsystem the command runs on
  private final Intake m_intake;
  private boolean extended;

  public ChangeIsExtended(Intake subsystem, boolean extended) {
    m_intake = subsystem;
    this.extended = extended;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.setPivotExtended(extended);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
