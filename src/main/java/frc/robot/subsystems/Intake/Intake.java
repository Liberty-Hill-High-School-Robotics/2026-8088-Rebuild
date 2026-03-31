package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private boolean isExtended = false;

  public Intake(IntakeIO io) {
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
    Logger.processInputs("Intake", inputs);
    io.updateInputs(inputs);
    io.resetIntakePivotEncoder();

    if (isExtended) {
      pivotOut();
    } else {
      pivotIn();
    }

    Logger.recordOutput("Intake/IsExtended", isExtended);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
    // Mostly used for debug and such
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // Should include run/stop/run back, etc.
  public void pivotIn() {
    io.intakePivotIn();
  }

  public void pivotOut() {
    io.intakePivotOut();
  }

  public void pivotStop() {
    io.intakePiviotStop();
  }

  public void setPivotExtended(boolean extended) {
    isExtended = extended;
  }

  public void intakeIn() {
    io.intakeIn();
  }

  public void eject() {
    io.eject();
  }

  public void intakeStop() {
    io.intakeStop();
  }
}
