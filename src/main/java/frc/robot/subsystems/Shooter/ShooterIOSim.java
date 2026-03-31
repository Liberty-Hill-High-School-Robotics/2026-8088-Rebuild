package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.MotorSpeeds;

public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim frontSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getNeoVortex(2), MotorSpeeds.shooterFrontMOI, 1),
          DCMotor.getNeoVortex(2),
          0.004);
  private final FlywheelSim backSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getNeoVortex(2), MotorSpeeds.shooterBackMOI, 1),
          DCMotor.getNeoVortex(2),
          0.004);

  private PIDController frontPid = new PIDController(0.02, 0, 0.0);
  private PIDController backPid = new PIDController(0.02, 0, 0.0);
  private SimpleMotorFeedforward frontFF = new SimpleMotorFeedforward(0, .0017699115044248);
  private SimpleMotorFeedforward backFF = new SimpleMotorFeedforward(0, .0017699115044248);

  private boolean closedLoop = false;
  private double frontAppliedVolts = 0.0;
  private double backAppliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (closedLoop) {
      frontAppliedVolts =
          MathUtil.clamp(
              frontPid.calculate(frontSim.getAngularVelocityRPM())
                  + frontFF.calculate(frontSim.getAngularVelocityRPM()),
              -12.0,
              12.0);
      frontSim.setInputVoltage(frontAppliedVolts);
    }

    frontSim.update(0.02);

    if (closedLoop) {
      backAppliedVolts =
          MathUtil.clamp(
              backPid.calculate(backSim.getAngularVelocityRPM())
                  + backFF.calculate(backSim.getAngularVelocityRPM()),
              -12.0,
              12.0);
      backSim.setInputVoltage(backAppliedVolts);
    }

    backSim.update(0.02);

    inputs.frontVelocity = frontSim.getAngularVelocityRPM();
    inputs.frontAppliedVolts = frontAppliedVolts;
    inputs.frontCurrentAmps = new double[] {frontSim.getCurrentDrawAmps()};

    inputs.backVelocity = backSim.getAngularVelocityRPM();
    inputs.backAppliedVolts = backAppliedVolts;
    inputs.backCurrentAmps = new double[] {backSim.getCurrentDrawAmps()};
  }

  @Override
  public void setVelocity(double velocity, double backingVelocity) {
    closedLoop = true;
    frontPid.setSetpoint(velocity);
    backPid.setSetpoint(backingVelocity);
  }

  @Override
  public void stop() {
    closedLoop = false;
    frontAppliedVolts = 0.0;
    backAppliedVolts = 0.0;
    frontSim.setInputVoltage(0.0);
    backSim.setInputVoltage(0.0);
  }
}
