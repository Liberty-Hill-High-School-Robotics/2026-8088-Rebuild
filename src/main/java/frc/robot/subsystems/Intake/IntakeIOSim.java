package frc.robot.subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.MotorSpeeds;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSim implements IntakeIO {
  private final SingleJointedArmSim intakePivotSim =
      new SingleJointedArmSim(
          LinearSystemId.createSingleJointedArmSystem(
              DCMotor.getNeoVortex(1), MotorSpeeds.kIntakePivotMOI, 1),
          DCMotor.getNeoVortex(1),
          1.0,
          Units.inchesToMeters(14.0),
          Units.degreesToRadians(0.0),
          Units.degreesToRadians(200.0),
          true,
          Units.degreesToRadians(0.0),
          new double[] {0.004, 0.004});

  private final FlywheelSim intakeSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), MotorSpeeds.kIntakeMOI, 1),
          DCMotor.getNeoVortex(1),
          0.004);

  private PIDController intakePivotController = new PIDController(.001, 0.0, 0.0);
  private PIDController intakeController = new PIDController(0.02, 0, 0.0);

  private ArmFeedforward intakePivotFF = new ArmFeedforward(0.0, 1.377, 0.0017699115044248);
  private SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(0.0, 0.0017699115044248);

  private boolean intakePivotClosedLoop = false;
  private boolean intakeClosedLoop = false;
  private double intakePivotAppliedVolts = 0.0;
  private double intakeAppliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (intakePivotClosedLoop) {
      intakePivotAppliedVolts =
          MathUtil.clamp(
              intakePivotController.calculate(intakePivotSim.getAngleRads())
                  + intakePivotFF.calculate(
                      intakePivotSim.getAngleRads(), 0), // TODO: does not work fix later
              -12.0,
              12.0);
      intakePivotSim.setInputVoltage(intakePivotAppliedVolts);
    }

    intakePivotSim.update(0.02);

    if (intakeClosedLoop) {
      intakeAppliedVolts =
          MathUtil.clamp(
              intakeController.calculate(intakeSim.getAngularVelocityRPM())
                  + intakeFF.calculate(intakeSim.getAngularVelocityRPM()),
              -12.0,
              12.0);
      intakeSim.setInputVoltage(intakeAppliedVolts);
    }

    intakeSim.update(0.02);

    inputs.intakeVelocity = intakeSim.getAngularVelocityRPM();
    inputs.intakeAppliedVolts = intakeAppliedVolts;
    inputs.intakeCurrentAmps = intakeSim.getCurrentDrawAmps();

    inputs.intakePositionRad = intakePivotSim.getAngleRads();
    inputs.intakePivotAppliedVolts = intakePivotAppliedVolts;
    inputs.intakePivotCurrentAmps = intakePivotSim.getCurrentDrawAmps();

    inputs.intakePivotAtLimit = intakePivotSim.getAngleRads() <= Units.rotationsToRadians(0.01);
  }

  // move the intake to its outward position
  @Override
  public void intakePivotOut() {
    intakePivotClosedLoop = true;
    intakePivotController.setSetpoint(Units.rotationsToRadians(Constants.kIntakePiviotExtendedLim));
    Logger.recordOutput(
        "Intake/Pivot/Setpoint", Units.rotationsToRadians(Constants.kIntakePiviotExtendedLim));
  }

  // move the intake to its inward position
  @Override
  public void intakePivotIn() {
    intakePivotClosedLoop = true;
    intakePivotController.setSetpoint(Units.rotationsToRadians(0));
    Logger.recordOutput("Intake/Pivot/Setpoint", Units.rotationsToRadians(0));
  }

  // stop moving the intake
  @Override
  public void intakePiviotStop() {
    intakePivotClosedLoop = false;
    intakePivotAppliedVolts = 0.0;
    intakePivotSim.setInputVoltage(0.0);
  }

  // Reset the intake pivot encoder to 0 when limit pressed. Does nothing in sim
  @Override
  public void resetIntakePivotEncoder() {}

  // intake fuel
  @Override
  public void intakeIn() {
    intakeClosedLoop = true;
    intakeController.setSetpoint(MotorSpeeds.kIntakeSpeed);
  }

  // eject fuel
  @Override
  public void eject() {
    intakeClosedLoop = true;
    intakeController.setSetpoint(-MotorSpeeds.kIntakeSpeed);
  }

  // Stop intaking
  @Override
  public void intakeStop() {
    intakeClosedLoop = false;
    intakeAppliedVolts = 0;
    intakeSim.setInputVoltage(0.0);
  }
}
