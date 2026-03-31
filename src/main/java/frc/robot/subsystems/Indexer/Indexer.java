package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorSpeeds;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        break;
      case SIM:
        break;
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Put smartdashboard stuff, check for limit switches
    Logger.processInputs("Index", inputs);
    io.updateInputs(inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
    // Mostly used for debug and such
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // Should include run/stop/run back, etc.

  public void indexToShooter() {
    double setpoint;
    if (SmartDashboard.getBoolean("Drive On Target Hub", true)) {
      io.setVelocity(MotorSpeeds.kIndexSpeed);
      setpoint = MotorSpeeds.kIndexSpeed;
    } else {
      setpoint = 0;
    }

    // Log indexer setpoint
    Logger.recordOutput("Indexer/Setpoint", setpoint, "rpm");
  }

  /** Stops the indexer. */
  public void indexStop() {
    io.stop();
  }
}
