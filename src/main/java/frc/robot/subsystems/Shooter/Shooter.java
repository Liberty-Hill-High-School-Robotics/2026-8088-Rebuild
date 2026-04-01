package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private double frontVelocity = 0;
  private double backVelocity = 0;

  private double testingPoint = 3000;
  private double testBackingRatio = .3;

  public Shooter(ShooterIO io) {
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
    Logger.processInputs("Shooter", inputs);
    io.updateInputs(inputs);

    SmartDashboard.putNumber("Shooter/TestBackinRatio", testBackingRatio);
    SmartDashboard.putNumber("Shooter/Front/TestVelocity", testingPoint);
    SmartDashboard.putNumber("Shooter/Back/TestVelocity", testingPoint * testBackingRatio);

    // Log flywheel setpoint
    Logger.recordOutput("Shooter/Front/Setpoint", frontVelocity, "rpm");
    Logger.recordOutput("Shooter/Back/Setpoint", backVelocity, "rpm");

    boolean frontAtSpeed = MathUtil.isNear(frontVelocity, inputs.frontVelocity, 350);
    boolean backAtSpeed = MathUtil.isNear(backVelocity, inputs.backVelocity, 350);

    SmartDashboard.putBoolean("Shooter/Front/AtSpeed", frontAtSpeed);
    SmartDashboard.putBoolean("Shooter/Back/AtSpeed", backAtSpeed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
    // Mostly used for debug and such
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // Should include run/stop/run back, etc.

  public void shootFromDistance() {
    double distance = SmartDashboard.getNumber("Distance to Target Hub", 0);
    frontVelocity =
        179.836
            + (595.497 * distance)
            + (264.868
                * Math.pow(distance, 2)); // TODO: equation to convert from distance to shooter RPM
    double backingRatio =
        4.5667
            - (2.51262 * distance)
            + (0.332342
                * Math.pow(
                    distance, 2)); // TODO: equation to convert fron distance to backing ratio
    backVelocity = frontVelocity * backingRatio;

    frontVelocity = MathUtil.clamp(frontVelocity, -6700, 6700);
    backVelocity = MathUtil.clamp(backVelocity, -6700, 6700);

    io.setVelocity(frontVelocity, backVelocity);
  }

  public void shootFromDistanceAirMail() {
    double distance = SmartDashboard.getNumber("Distance to Target Air Mail", 0);
    frontVelocity =
        179.836
            + (595.497 * distance)
            + (264.868
                * Math.pow(distance, 2)); // TODO: equation to convert from distance to shooter RPM
    double backingRatio =
        4.5667
            - (2.51262 * distance)
            + (0.332342
                * Math.pow(
                    distance, 2)); // TODO: equation to convert fron distance to backing ratio
    backVelocity = frontVelocity * backingRatio;

    frontVelocity = MathUtil.clamp(frontVelocity, -6700, 6700);
    backVelocity = MathUtil.clamp(backVelocity, -6700, 6700);

    io.setVelocity(frontVelocity, backVelocity);
  }

  public void shootAtTestingSpeed() {
    frontVelocity = testingPoint;
    backVelocity = frontVelocity * testBackingRatio;
    frontVelocity = MathUtil.clamp(frontVelocity, -6700, 6700);
    backVelocity = MathUtil.clamp(backVelocity, -6700, 6700);
    io.setVelocity(testingPoint, backVelocity);
  }

  public void changeTestVelocity(double amount) {
    testingPoint += amount;
  }

  public void changeTestBackingRatio(double amount) {
    testBackingRatio += amount;
  }

  /** Stops the flywheel. */
  public void shooterStop() {
    io.stop();
    frontVelocity = 0;
    backVelocity = 0;
  }
}
