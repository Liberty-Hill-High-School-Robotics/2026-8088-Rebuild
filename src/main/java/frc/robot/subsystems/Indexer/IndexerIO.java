package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    double indexVelocity = 0.0;
    double indexAppliedVolts = 0.0;
    double[] indexCurrentAmps = new double[] {};
  }

  // Updates the set of loggable inputs.
  public default void updateInputs(IndexerIOInputs inputs) {}

  // Spin the shooter rollers at a specified rpm
  public default void setVelocity(double velocity) {}

  // Stop ther shooter motors
  public default void stop() {}
}
