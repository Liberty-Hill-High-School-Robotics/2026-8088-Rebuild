package frc.robot.subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorSpeeds;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private boolean isExtended = false;
  private double pivotSetpoint = 0;
  private double intakeSetpoint = 0;

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
    Logger.recordOutput("Intake/PivotSetpoint", pivotSetpoint);
    Logger.recordOutput("Intake/IsExtended", isExtended);

    Logger.recordOutput("Intake/IntakeSetpoint", intakeSetpoint);
    Logger.recordOutput("Intake/isIntakeJammed", getIsIntakeJammed());
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
    pivotSetpoint = Constants.kIntakePiviotRetractedLim;
  }

  public void pivotOut() {
    io.intakePivotOut();
    pivotSetpoint = Constants.kIntakePiviotExtendedLim;
  }

  public void pivotStop() {
    io.intakePiviotStop();
  }

  public void setPivotExtended(boolean extended) {
    isExtended = extended;
  }

  public void intakeIn() {
    io.intakeIn();
    intakeSetpoint = MotorSpeeds.kIntakeSpeed;
  }

  public void eject() {
    io.eject();
    intakeSetpoint = -MotorSpeeds.kIntakeSpeed;
  }

  public void intakeStop() {
    io.intakeStop();
  }

  public boolean getIsIntakeJammed() {
    return !MathUtil.isNear(intakeSetpoint, inputs.intakeVelocity, 500);
  }
}
