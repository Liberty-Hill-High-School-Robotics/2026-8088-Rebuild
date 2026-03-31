package frc.robot.subsystems.Indexer;

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

public class IndexerIOSparkFlex implements IndexerIO {
  // Hardware
  private final SparkBase indexLeadMotor;
  private final SparkBase indexFollowMotor;

  private final RelativeEncoder indexEncoder;
  private final SparkClosedLoopController indexController;

  public IndexerIOSparkFlex() {
    indexLeadMotor = new SparkFlex(CanIDs.kIndexLeadMotor, MotorType.kBrushless);
    indexFollowMotor = new SparkFlex(CanIDs.kIndexLeadMotor, MotorType.kBrushless);

    indexEncoder = indexLeadMotor.getEncoder();

    // Config Leader
    SparkFlexConfig indexLeadConfig = new SparkFlexConfig();
    indexLeadConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(MotorSpeeds.kIndexerP)
        .i(MotorSpeeds.kIndexerI)
        .d(MotorSpeeds.kIndexerD)
        .feedForward
        .kS(MotorSpeeds.kIndexerS)
        .kV(MotorSpeeds.kIndexerV);
    // https://www.chiefdelphi.com/t/psa-rev-spark-default-velocity-filtering-is-still-really-bad-for-flywheels/514567
    indexLeadConfig
        .encoder
        .uvwMeasurementPeriod(8)
        .quadratureAverageDepth(2)
        .quadratureMeasurementPeriod(8);

    tryUntilOk(
        indexLeadMotor,
        5,
        () ->
            indexLeadMotor.configure(
                indexLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Config Follower
    SparkFlexConfig indexFollowConfig = new SparkFlexConfig();
    indexFollowConfig.follow(indexLeadMotor).inverted(true);

    tryUntilOk(
        indexFollowMotor,
        5,
        () ->
            indexFollowMotor.configure(
                indexFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    indexController = indexLeadMotor.getClosedLoopController();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexVelocity = indexEncoder.getVelocity();
    inputs.indexAppliedVolts =
        indexLeadMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.indexCurrentAmps =
        new double[] {indexLeadMotor.getOutputCurrent(), indexFollowMotor.getOutputCurrent()};
  }

  // Spin the shooter rollers at a specified rpm
  @Override
  public void setVelocity(double velocity) {
    indexController.setSetpoint(velocity, ControlType.kVelocity);
  }

  // Stop ther shooter motors
  @Override
  public void stop() {
    indexLeadMotor.stopMotor();
  }
}
