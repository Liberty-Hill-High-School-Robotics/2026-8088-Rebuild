package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> poseSupplier;
  private boolean isSpinUp = false;

  private double frontVelocity = 0;
  private double backVelocity = 0;

  private double testingPoint = 3000;
  private double testBackingRatio = .3;

  public Shooter(
      ShooterIO io, java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> poseSupplier) {
    this.io = io;
    this.poseSupplier = poseSupplier;

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

    boolean frontAtSpeed = MathUtil.isNear(frontVelocity + 50, inputs.frontVelocity, 75);
    boolean backAtSpeed = MathUtil.isNear(backVelocity + 50, inputs.backVelocity, 75);

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

    /*
    frontVelocity = MotorSpeeds.kDistanceToRPMMap.get(distance);
    double backingRatio = MotorSpeeds.kDistanceToBacking.get(distance);
    backVelocity = frontVelocity * backingRatio;
    */
    frontVelocity =
        -7010.64
            + (13383.4 * distance)
            - (8682.62 * Math.pow(distance, 2))
            + (2835.12 * Math.pow(distance, 3))
            - (454.093 * Math.pow(distance, 4))
            + (28.4278 * Math.pow(distance, 5));
    double backingRatio =
        22.6983
            - 32.6589 * distance
            + 19.6486 * Math.pow(distance, 2)
            - 5.78241 * Math.pow(distance, 3)
            + 0.831055 * Math.pow(distance, 4)
            - 0.0467505 * Math.pow(distance, 5);
    backingRatio = MathUtil.clamp(backingRatio, 0.5, Double.POSITIVE_INFINITY);
    backVelocity = frontVelocity * backingRatio;

    frontVelocity = MathUtil.clamp(frontVelocity, -6700, 6700);
    backVelocity = MathUtil.clamp(backVelocity, -6700, 6700);

    io.setVelocity(frontVelocity, backVelocity);
  }

  public void rampToVelocitySlow() {
    double distance = SmartDashboard.getNumber("Distance to Target Hub", 0);
    /*
    frontVelocity = MotorSpeeds.kDistanceToRPMMap.get(distance);
    double backingRatio = MotorSpeeds.kDistanceToBacking.get(distance);
    backVelocity = frontVelocity * backingRatio;
    */

    frontVelocity =
        -7010.64
            + (13383.4 * distance)
            - (8682.62 * Math.pow(distance, 2))
            + (2835.12 * Math.pow(distance, 3))
            - (454.093 * Math.pow(distance, 4))
            + (28.4278 * Math.pow(distance, 5));
    double backingRatio =
        22.6983
            - 32.6589 * distance
            + 19.6486 * Math.pow(distance, 2)
            - 5.78241 * Math.pow(distance, 3)
            + 0.831055 * Math.pow(distance, 4)
            - 0.0467505 * Math.pow(distance, 5);
    backingRatio = MathUtil.clamp(backingRatio, 0.5, Double.POSITIVE_INFINITY);
    backVelocity = frontVelocity * backingRatio;

    frontVelocity = MathUtil.clamp(frontVelocity, -6700, 6700);
    backVelocity = MathUtil.clamp(backVelocity, -6700, 6700);

    frontVelocity += 5;
    backVelocity += 5;

    frontVelocity = MathUtil.clamp(frontVelocity, -6705, 6705);
    backVelocity = MathUtil.clamp(backVelocity, -6705, 6705);

    io.rampToVelocitySlow(frontVelocity, backVelocity);
  }

  public void shootFromDistanceAirMail() {
    double distance = SmartDashboard.getNumber("Distance to Target Air Mail", 0);
    /*
    frontVelocity = MotorSpeeds.kDistanceToRPMMapMail.get(distance);
    double backingRatio = MotorSpeeds.kDistanceToBackingMail.get(distance);
    backVelocity = frontVelocity * backingRatio;
    */

    frontVelocity =
        -7010.64
            + (13383.4 * distance)
            - (8682.62 * Math.pow(distance, 2))
            + (2835.12 * Math.pow(distance, 3))
            - (454.093 * Math.pow(distance, 4))
            + (28.4278 * Math.pow(distance, 5));
    double backingRatio =
        22.6983
            - 32.6589 * distance
            + 19.6486 * Math.pow(distance, 2)
            - 5.78241 * Math.pow(distance, 3)
            + 0.831055 * Math.pow(distance, 4)
            - 0.0467505 * Math.pow(distance, 5);
    backingRatio = MathUtil.clamp(backingRatio, 0.5, Double.POSITIVE_INFINITY);
    backVelocity = frontVelocity * backingRatio;

    frontVelocity = MathUtil.clamp(frontVelocity, -6700, 6700);
    backVelocity = MathUtil.clamp(backVelocity, -6700, 6700);

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

  public void setIsSpinup(boolean doSpinup) {
    isSpinUp = doSpinup;
  }

  public boolean getIsSpinup() {
    return isSpinUp;
  }

  public double[] getVelocities() {
    return new double[] {inputs.frontVelocity, inputs.backVelocity};
  }

  /** Stops the flywheel. */
  public void shooterStop() {
    io.stop();
    frontVelocity = 0;
    backVelocity = 0;
  }
}
