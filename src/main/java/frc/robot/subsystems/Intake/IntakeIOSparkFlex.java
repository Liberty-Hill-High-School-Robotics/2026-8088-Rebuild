package frc.robot.subsystems.Intake;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.MotorSpeeds;

public class IntakeIOSparkFlex implements IntakeIO {
  // Hardware
  private final SparkBase intakePivotMotor;
  private final SparkBase intakeMotor;
  private final RelativeEncoder intakePivotEncoder;
  private final RelativeEncoder intakeEncoder;

  private final SparkClosedLoopController intakePivotController;
  private final SparkClosedLoopController intakeController;

  public IntakeIOSparkFlex() {
    intakePivotMotor = new SparkFlex(CanIDs.kIntakePiviotMotor, MotorType.kBrushless);
    intakeMotor = new SparkFlex(CanIDs.kIntakeMotor, MotorType.kBrushless);

    intakePivotEncoder = intakePivotMotor.getEncoder();
    intakeEncoder = intakeMotor.getEncoder();

    // Config Intake Pivot
    SparkFlexConfig intakePivotConfig = new SparkFlexConfig();
    intakePivotConfig
        .inverted(true)
        .smartCurrentLimit(35)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(MotorSpeeds.kIntakePivotP)
        .i(MotorSpeeds.kIntakePivotI)
        .d(MotorSpeeds.kIntakePivotD)
        .feedForward
        .kS(MotorSpeeds.kIntakePivotS)
        .kV(MotorSpeeds.kIntakeV);
    intakePivotConfig.limitSwitch.reverseLimitSwitchPosition(0);

    tryUntilOk(
        intakePivotMotor,
        5,
        () ->
            intakePivotMotor.configure(
                intakePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Config Intake
    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig
        .inverted(true)
        .smartCurrentLimit(45)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(MotorSpeeds.kIntakeP)
        .i(MotorSpeeds.kIntakeI)
        .d(MotorSpeeds.kIntakeD)
        .feedForward
        .kS(MotorSpeeds.kIntakeS)
        .kV(MotorSpeeds.kIntakeV);

    tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    intakePivotController = intakePivotMotor.getClosedLoopController();
    intakeController = intakeMotor.getClosedLoopController();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakePositionRad = intakePivotEncoder.getPosition();
    inputs.intakePivotVelocity = intakePivotEncoder.getVelocity();
    inputs.intakePivotAppliedVolts =
        intakePivotMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.intakePivotCurrentAmps = intakePivotMotor.getOutputCurrent();

    inputs.intakeVelocity = intakeEncoder.getVelocity();
    inputs.intakeAppliedVolts =
        intakeMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();

    inputs.intakePivotAtLimit = intakePivotMotor.getReverseLimitSwitch().isPressed();
  }

  // move the intake to its outward position
  @Override
  public void intakePivotOut() {
    if (intakePivotMotor.getEncoder().getPosition() < Constants.kIntakePiviotExtendedLim) {
      intakePivotController.setSetpoint(MotorSpeeds.kIntakePivotSpeed, ControlType.kVelocity);
    } else {
      intakePivotMotor.stopMotor();
    }
  }

  // move the intake to its inward position
  @Override
  public void intakePivotIn() {
    if (intakePivotMotor.getEncoder().getPosition() > Constants.kIntakePiviotRetractedLim
        && !intakePivotMotor.getReverseLimitSwitch().isPressed()) {
      intakePivotController.setSetpoint(-MotorSpeeds.kIntakePivotSpeed, ControlType.kVelocity);
    } else {
      intakePivotMotor.stopMotor();
    }
  }

  // stop moving the intake
  @Override
  public void intakePiviotStop() {
    intakePivotMotor.stopMotor();
  }

  // Reset the intake pivot encoder to 0 when limit pressed
  @Override
  public void resetIntakePivotEncoder() {
    if (intakePivotMotor.getReverseLimitSwitch().isPressed()) {
      intakePivotEncoder.setPosition(0);
    }
  }

  // intake fuel
  @Override
  public void intakeIn() {
    intakeController.setSetpoint(MotorSpeeds.kIntakeSpeed, ControlType.kVelocity);
  }

  // eject fuel
  @Override
  public void eject() {
    intakeController.setSetpoint(-MotorSpeeds.kIntakeSpeed, ControlType.kVelocity);
  }

  // Stop intaking
  @Override
  public void intakeStop() {
    intakeMotor.stopMotor();
  }
}
