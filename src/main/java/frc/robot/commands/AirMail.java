package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Indexer.IndexToShooterDumb;
import frc.robot.commands.Shooter.ShootAtSpeedAirMail;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.Shooter;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class AirMail extends ParallelCommandGroup {

  public AirMail(Indexer m_indexer, Shooter m_shooter) {
    addCommands(
        new IndexToShooterDumb(m_indexer), // Pass Balls to Shooter skip checks
        new ShootAtSpeedAirMail(m_shooter)); // spin up shooter
  }

  @Override
  public boolean runsWhenDisabled() {

    return false;
  }
}
