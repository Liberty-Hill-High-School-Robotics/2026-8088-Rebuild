package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    double intakePositionRad = 0.0;
    double intakePivotAppliedVolts = 0.0;
    double intakePivotCurrentAmps = 0.0;

    double intakeVelocity = 0.0;
    double intakeAppliedVolts = 0.0;
    double intakeCurrentAmps = 0.0;

    boolean intakePivotAtLimit = false;
  }

  // Updates the set of loggable inputs.
  public default void updateInputs(IntakeIOInputs inputs) {}

  // move the intake to its outward position
  public default void intakePivotOut() {}

  // move the intake to its inward position
  public default void intakePivotIn() {}

  // stop moving the intake
  public default void intakePiviotStop() {}

  // Reset the intake pivot encoder to 0 when limit pressed
  public default void resetIntakePivotEncoder() {}

  // intake fuel
  public default void intakeIn() {}

  // eject fuel
  public default void eject() {}

  // Stop intaking
  public default void intakeStop() {}
}
