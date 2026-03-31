package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    double frontVelocity = 0.0;
    double frontAppliedVolts = 0.0;
    double[] frontCurrentAmps = new double[] {};

    double backVelocity = 0.0;
    double backAppliedVolts = 0.0;
    double[] backCurrentAmps = new double[] {};
  }

  // Updates the set of loggable inputs.
  public default void updateInputs(ShooterIOInputs inputs) {}

  // Spin the shooter rollers at a specified rpm
  public default void setVelocity(double velocity, double backingVelocity) {}

  // Stop ther shooter motors
  public default void stop() {}
}
