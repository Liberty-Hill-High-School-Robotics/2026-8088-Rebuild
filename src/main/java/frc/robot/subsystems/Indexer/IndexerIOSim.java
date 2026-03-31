package frc.robot.subsystems.Indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.MotorSpeeds;

public class IndexerIOSim implements IndexerIO {
  private final FlywheelSim indexSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getNeoVortex(2), MotorSpeeds.shooterBackMOI, 1),
          DCMotor.getNeoVortex(2),
          0.004);

  private PIDController indexPID = new PIDController(0.02, 0, 0.0);
  private SimpleMotorFeedforward indexFF = new SimpleMotorFeedforward(0, .0017699115044248);

  private boolean closedLoop = false;
  private double indexAppliedVolts = 0.0;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    if (closedLoop) {
      indexAppliedVolts =
          MathUtil.clamp(
              indexPID.calculate(indexSim.getAngularVelocityRPM())
                  + indexFF.calculate(indexSim.getAngularVelocityRPM()),
              -12.0,
              12.0);
      indexSim.setInputVoltage(indexAppliedVolts);
    }

    indexSim.update(0.02);

    inputs.indexVelocity = indexSim.getAngularVelocityRPM();
    inputs.indexAppliedVolts = indexAppliedVolts;
    inputs.indexCurrentAmps = new double[] {indexSim.getCurrentDrawAmps()};
  }

  @Override
  public void setVelocity(double velocity) {
    closedLoop = true;
    indexPID.setSetpoint(velocity);
  }

  @Override
  public void stop() {
    closedLoop = false;
    indexAppliedVolts = 0.0;
    indexSim.setInputVoltage(0.0);
  }
}
