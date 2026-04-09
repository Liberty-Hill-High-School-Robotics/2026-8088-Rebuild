package frc.robot.commands.Shooter;

import static frc.robot.util.InZone.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.subsystems.Shooter.Shooter;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class RampToVelocitySlow extends Command {
  // The subsystem the command runs on
  private final Shooter m_shooter;
  private java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> poseSupplier;

  public RampToVelocitySlow(
      Shooter subsystem,
      java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> poseSupplier) {
    m_shooter = subsystem;
    this.poseSupplier = poseSupplier;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    MotorSpeeds.kFrontLimiter.reset(m_shooter.getVelocities()[0]);
    MotorSpeeds.kBackLimiter.reset(m_shooter.getVelocities()[1]);
  }

  @Override
  public void execute() {
    if (isInAllianceZone(poseSupplier.get().getX()) && m_shooter.getIsSpinup()) {
      m_shooter.rampToVelocitySlow();
    } else {
      MotorSpeeds.kFrontLimiter.reset(m_shooter.getVelocities()[0]);
      MotorSpeeds.kBackLimiter.reset(m_shooter.getVelocities()[1]);
      m_shooter.shooterStop();
    }
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
