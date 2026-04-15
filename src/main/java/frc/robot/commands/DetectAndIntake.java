package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class DetectAndIntake extends ParallelCommandGroup {

  public DetectAndIntake(
      Drive m_drive,
      /*Intake m_intake,
      Indexer m_indexer,
      */
      DoubleSupplier averageYawSupplier,
      DoubleSupplier forwardSupplier) {
    PIDController targetPID = new PIDController(.02, 0, 0);
    addCommands(
        DriveCommands.joystickDriveRobot(
            m_drive,
            forwardSupplier,
            () -> 0,
            () -> -targetPID.calculate(0, averageYawSupplier.getAsDouble())));
    // new IntakeIn(m_intake),
    // new IndexerEject(m_indexer));
    targetPID.close();
  }

  @Override
  public boolean runsWhenDisabled() {

    return false;
  }
}
