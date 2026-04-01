package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class ShootAtSpeedAirMail extends Command {
  // The subsystem the command runs on
  private final Shooter m_shooter;

  public ShootAtSpeedAirMail(Shooter subsystem) {
    m_shooter = subsystem;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_shooter.shootFromDistanceAirMail();
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.shooterStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
