package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorSpeeds;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private double setpoint = 0;

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

    Logger.recordOutput("Indexer/Setpoint", setpoint, "rpm");
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
    // Mostly used for debug and such
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // Should include run/stop/run back, etc.

  public void indexToShooterSmart(boolean isAirmail) {
    if (isAirmail) {
      if (SmartDashboard.getBoolean("Drive On Target Mail", true)) {
        if (SmartDashboard.getBoolean("Shooter/Front/AtSpeed", true)
            && SmartDashboard.getBoolean("Shooter/Back/AtSpeed", true)) {
          io.setVelocity(MotorSpeeds.kIndexSpeed);
          setpoint = MotorSpeeds.kIndexSpeed;
        } else {
          indexStop();
        }
      } else {
        indexStop();
      }
    } else {
      if (SmartDashboard.getBoolean("Drive On Target Hub", true)) {
        if (SmartDashboard.getBoolean("Shooter/Front/AtSpeed", true)
            && SmartDashboard.getBoolean("Shooter/Back/AtSpeed", true)) {
          io.setVelocity(MotorSpeeds.kIndexSpeed);
          setpoint = MotorSpeeds.kIndexSpeed;
        } else {
          indexStop();
        }
      } else {
        indexStop();
      }
    }
  }

  public void indexToShooterDumb() {
    io.setVelocity(MotorSpeeds.kIndexSpeed);
    setpoint = MotorSpeeds.kIndexSpeed;
  }

  /** Stops the indexer. */
  public void indexStop() {
    io.stop();
    setpoint = 0;
  }

  public void indexReverse() {
    io.setVelocity(-MotorSpeeds.kIndexSpeed);
    setpoint = -MotorSpeeds.kIndexSpeed;
  }
}
