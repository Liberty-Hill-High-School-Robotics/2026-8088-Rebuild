package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer.Indexer;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class IndexerEject extends Command {
  // The subsystem the command runs on
  private final Indexer m_indexer;

  public IndexerEject(Indexer subsystem) {
    m_indexer = subsystem;
    addRequirements(m_indexer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_indexer.indexReverse();
  }

  @Override
  public void end(boolean interrupted) {
    m_indexer.indexStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
