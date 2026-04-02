package frc.robot.subsystems.Shooter;

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
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.MotorSpeeds;

public class ShooterIOSparkFlex implements ShooterIO {
  // Hardware
  private final SparkBase shooterFrontLeadMotor;
  private final SparkBase shooterFrontFollowMotor;
  private final SparkBase shooterBackLeadMotor;
  private final SparkBase shooterBackFollowMotor;
  private final RelativeEncoder shooterFrontEncoder;
  private final RelativeEncoder shooterBackEncoder;

  private final SparkClosedLoopController shooterFrontController;
  private final SparkClosedLoopController shooterBackController;

  public ShooterIOSparkFlex() {
    shooterFrontLeadMotor = new SparkFlex(CanIDs.kShooterFrontLeadMotor, MotorType.kBrushless);
    shooterFrontFollowMotor = new SparkFlex(CanIDs.kShooterFrontFollowMotor, MotorType.kBrushless);
    shooterBackLeadMotor = new SparkFlex(CanIDs.kShooterBackLeadMotor, MotorType.kBrushless);
    shooterBackFollowMotor = new SparkFlex(CanIDs.kShooterBackFollowMotor, MotorType.kBrushless);

    shooterFrontEncoder = shooterFrontLeadMotor.getEncoder();
    shooterBackEncoder = shooterBackLeadMotor.getEncoder();

    // Config front Leader
    SparkFlexConfig frontLeadConfig = new SparkFlexConfig();
    frontLeadConfig
        .smartCurrentLimit(50)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(MotorSpeeds.kShooterFrontP)
        .i(MotorSpeeds.kShooterFrontI)
        .d(MotorSpeeds.kShooterFrontD)
        .feedForward
        .kS(MotorSpeeds.kShooterFrontS)
        .kV(MotorSpeeds.kShooterFrontV);
    // https://www.chiefdelphi.com/t/psa-rev-spark-default-velocity-filtering-is-still-really-bad-for-flywheels/514567
    frontLeadConfig
        .encoder
        .uvwMeasurementPeriod(8)
        .quadratureAverageDepth(2)
        .quadratureMeasurementPeriod(8);

    tryUntilOk(
        shooterFrontLeadMotor,
        5,
        () ->
            shooterFrontLeadMotor.configure(
                frontLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Config front Follower
    SparkFlexConfig frontFollowConfig = new SparkFlexConfig();
    frontFollowConfig.follow(shooterFrontLeadMotor, true).smartCurrentLimit(50);

    tryUntilOk(
        shooterFrontFollowMotor,
        5,
        () ->
            shooterFrontFollowMotor.configure(
                frontFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Config back Leader
    SparkFlexConfig backLeadConfig = new SparkFlexConfig();
    backLeadConfig
        .smartCurrentLimit(50)
        .inverted(true)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(MotorSpeeds.kShooterBackP)
        .i(MotorSpeeds.kShooterBackI)
        .d(MotorSpeeds.kShooterBackD)
        .feedForward
        .kS(MotorSpeeds.kShooterBackS)
        .kV(MotorSpeeds.kShooterBackV);
    // https://www.chiefdelphi.com/t/psa-rev-spark-default-velocity-filtering-is-still-really-bad-for-flywheels/514567
    backLeadConfig
        .encoder
        .uvwMeasurementPeriod(8)
        .quadratureAverageDepth(2)
        .quadratureMeasurementPeriod(8);

    tryUntilOk(
        shooterBackLeadMotor,
        5,
        () ->
            shooterBackLeadMotor.configure(
                backLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Config back Follower
    SparkFlexConfig backFollowConfig = new SparkFlexConfig();
    backFollowConfig.follow(shooterBackLeadMotor, true).smartCurrentLimit(50);

    tryUntilOk(
        shooterBackFollowMotor,
        5,
        () ->
            shooterBackFollowMotor.configure(
                backFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    shooterFrontController = shooterFrontLeadMotor.getClosedLoopController();
    shooterBackController = shooterBackLeadMotor.getClosedLoopController();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.frontVelocity = shooterFrontEncoder.getVelocity();
    inputs.frontAppliedVolts =
        shooterFrontLeadMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.frontCurrentAmps =
        new double[] {
          shooterFrontLeadMotor.getOutputCurrent(), shooterFrontFollowMotor.getOutputCurrent()
        };

    inputs.backVelocity = shooterBackEncoder.getVelocity();
    inputs.backAppliedVolts =
        shooterBackLeadMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.backCurrentAmps =
        new double[] {
          shooterBackLeadMotor.getOutputCurrent(), shooterBackFollowMotor.getOutputCurrent()
        };
  }

  // Spin the shooter rollers at a specified rpm
  @Override
  public void setVelocity(double velocity, double backingVelocity) {
    shooterFrontController.setSetpoint(velocity, ControlType.kVelocity);
    shooterBackController.setSetpoint(backingVelocity, ControlType.kVelocity);
  }

  // Stop ther shooter motors
  @Override
  public void stop() {
    shooterFrontLeadMotor.stopMotor();
    shooterBackLeadMotor.stopMotor();
  }
}
