// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double kDriveShootingRatio = 0.2;

  public static final double kTargetAllowedError = 10;

  public static final double kIntakePiviotExtendedLim = 9.543945; // Motor rotations
  public static final double kIntakePiviotRetractedLim = .5; // Motor rotations

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class CanIDs {
    // can IDs

    // Drive
    public static final int kFLDrivingCAN = 1;
    public static final int kFRDrivingCAN = 2;
    public static final int kBLDrivingCAN = 3;
    public static final int kBRDrivingCAN = 4;

    public static final int kFLTurningCAN = 5;
    public static final int kFRTurningCAN = 6;
    public static final int kBLTurningCAN = 7;
    public static final int kBRTurningCAN = 8;
    public static final int kGyroID = 9;

    // Intake
    public static final int kIntakeMotor = 10;
    public static final int kIntakePiviotMotor = 11;

    // Index
    public static final int kIndexLeadMotor = 12;
    public static final int kIndexFollowMotor = 13;

    // Shooter
    public static final int kShooterFrontLeadMotor = 14;
    public static final int kShooterFrontFollowMotor = 15;
    public static final int kShooterBackLeadMotor = 16;
    public static final int kShooterBackFollowMotor = 17;

    // Climber TODO: no climber yet
    // public static final int kClimbMotor = 18;
    // public static final int kClimbPiviotMotor = 19;
  }

  // Speeds for motors and PID constants
  public static final class MotorSpeeds {

    // Shooter Front
    public static final double kShooterFrontP = 0.00034;
    public static final double kShooterFrontI = 0.0;
    public static final double kShooterFrontD = 0.0001;

    public static final double kShooterFrontS = 0;
    public static final double kShooterFrontV = 0.0017699115044248;

    // Shooter Back
    public static final double kShooterBackP = 0.0003;
    public static final double kShooterBackI = 0.0;
    public static final double kShooterBackD = 0.0001;

    public static final double kShooterBackS = 0;
    public static final double kShooterBackV = 0.0017699115044248;

    // Shooter MOI For SIM
    public static final double shooterFrontMOI = 0.003; // TODO: calc for real MOI
    public static final double shooterBackMOI = 0.003; // TODO: calc for real MOI

    // Intake
    public static final double kIntakeP = 0.0002;
    public static final double kIntakeI = 0.00001;
    public static final double kIntakeD = 0.0;

    public static final double kIntakeS = 0.0;
    public static final double kIntakeV = 0.0017699115044248;

    // Intake Pivot
    public static final double kIntakePivotP = 0.0002;
    public static final double kIntakePivotI = 0.000001;
    public static final double kIntakePivotD = 0.0000001;
    // https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control?q=tunning#manually-finding-kcos-and-ks-for-an-arm
    // Do first
    public static final double kIntakePivotS = 0.6215;
    public static final double kIntakePivotV = 0.0017699115044248;
    /*
    public static final double kIntakePivotCos = 1.2415;
    public static final double kIntakePivotCosRatio = 32.0000;
    */

    public static final double kIntakePivotSpeed = 500.0;

    // Intake MOI for SIM
    public static final double kIntakePivotMOI = 0.01; // TODO: calc for real MOI
    public static final double kIntakeMOI = 0.003; // TODO: calc for real MOI

    public static final double kIntakeSpeed = 5500.0; // speed to run intake motor RPM

    // Index
    public static final double kIndexerP = 0.00034;
    public static final double kIndexerI = 0.0;
    public static final double kIndexerD = 0.0;

    public static final double kIndexerS = 0.167;
    public static final double kIndexerV = 0.0017699115044248;

    public static final double kIndexMOI = 0.003; // TODO: calc for real MOI

    public static final double kIndexSpeed = 500.0; // speed to run index motor RPM

    public static InterpolatingDoubleTreeMap kDistanceToRPMMap = new InterpolatingDoubleTreeMap();

    static {
      kDistanceToRPMMap.put(1.163880614, 1300.0);
      kDistanceToRPMMap.put(1.720957, 1900.0);
      kDistanceToRPMMap.put(2.011616483, 2050.0);
      kDistanceToRPMMap.put(2.437264, 2400.0);
      kDistanceToRPMMap.put(2.828957109, 2800.0);
      kDistanceToRPMMap.put(3.002837776, 3000.0);
      kDistanceToRPMMap.put(3.303899, 3090.0);
      kDistanceToRPMMap.put(3.343351, 3100.0);
      kDistanceToRPMMap.put(3.689604834497496, 3450.0);
      kDistanceToRPMMap.put(4.019709447, 3900.0);
    }

    public static InterpolatingDoubleTreeMap kDistanceToBacking = new InterpolatingDoubleTreeMap();

    static {
      kDistanceToBacking.put(1.163880614, 1.6);
      kDistanceToBacking.put(1.720957, 1.0);
      kDistanceToBacking.put(2.011616483, 0.9);
      kDistanceToBacking.put(2.437264, 0.65);
      kDistanceToBacking.put(2.828957109, 0.56);
      kDistanceToBacking.put(3.002837776, 0.55);
      kDistanceToBacking.put(3.303899, 0.54);
      kDistanceToBacking.put(3.343351, 0.51);
      kDistanceToBacking.put(3.689604834497496, .4);
      kDistanceToBacking.put(4.019709447, 0.35);
    }

    public static InterpolatingDoubleTreeMap kDistanceToRPMMapMail =
        new InterpolatingDoubleTreeMap();

    static {
      kDistanceToRPMMapMail.put(1.163880614, 2000.0);
      kDistanceToRPMMapMail.put(1.720957, 2400.0);
      kDistanceToRPMMapMail.put(2.011616483, 2600.0);
      kDistanceToRPMMapMail.put(2.437264, 3000.0);
      kDistanceToRPMMapMail.put(2.828957109, 3200.0);
      kDistanceToRPMMapMail.put(3.002837776, 3400.0);
      kDistanceToRPMMapMail.put(3.303899, 3600.0);
      kDistanceToRPMMapMail.put(3.343351, 3700.0);
      kDistanceToRPMMapMail.put(4.019709447, 4100.0);
      kDistanceToRPMMapMail.put(6.0, 4500.0);
      kDistanceToRPMMapMail.put(8.0, 5000.0);
      kDistanceToRPMMapMail.put(10.714630449757905, 6000.0);
    }

    public static InterpolatingDoubleTreeMap kDistanceToBackingMail =
        new InterpolatingDoubleTreeMap();

    static {
      kDistanceToBackingMail.put(1.163880614, 1.0);
      kDistanceToBackingMail.put(1.6316, .8);
      kDistanceToBackingMail.put(2.011616483, 0.7);
      kDistanceToBackingMail.put(2.828957109, 0.3);
      kDistanceToBackingMail.put(3.002837776, 0.25);
      kDistanceToBackingMail.put(3.343351, 0.2);
      kDistanceToBackingMail.put(4.019709447, 0.15);
      kDistanceToBackingMail.put(6.0, 0.12);
      kDistanceToBackingMail.put(8.0, 0.11);
      kDistanceToBackingMail.put(10.714630449757905, 0.1);
    }

    public static final SlewRateLimiter kFrontLimiter =
        new SlewRateLimiter(500); // 500 rpm/s acceleration limit
    public static final SlewRateLimiter kBackLimiter =
        new SlewRateLimiter(500); // 500 rpm/s acceleration limit
  }

  public static final class OIConstants {
    // controller ports
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.2;
  }

  public static final class VisionConstants {
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static final Transform3d kFrontRobotToCam =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(7.11216728),
                Units.inchesToMeters(11.34720739)),
            new Rotation3d(0, Units.degreesToRadians(15), 0)); // TODO: get real numbers from CAD

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSideSingleTagStdDevs = VecBuilder.fill(3.5, 3.5, 8);
    public static final Matrix<N3, N1> kSideMultiTagStdDevs = VecBuilder.fill(0.3, 0.3, 1);

    public static final Transform3d kSideRobotToCam =
        new Transform3d(
            new Translation3d(0, Units.inchesToMeters(10.25), Units.inchesToMeters(10.5)),
            new Rotation3d(
                0,
                Units.degreesToRadians(15),
                Units.degreesToRadians(-90))); // TODO: get real numbers from CAD

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kFrontSingleTagStdDevs =
        VecBuilder.fill(5, 5, 10); // lower = more trust in vision
    public static final Matrix<N3, N1> kFrontMultiTagStdDevs = VecBuilder.fill(1, 1, 2);
  }

  public static final class FieldConstants {
    public static final Pose2d kBlueHubPose =
        new Pose2d(Units.inchesToMeters(181.56), Units.inchesToMeters(158.32), new Rotation2d());
    public static final Pose2d kRedHubPose =
        new Pose2d(Units.inchesToMeters(468.56), Units.inchesToMeters(158.32), new Rotation2d());

    public static final Pose2d kRedTopAirMail =
        new Pose2d(Units.inchesToMeters(559.34), Units.inchesToMeters(237.48), new Rotation2d());
    public static final Pose2d kRedLowAirMail =
        new Pose2d(Units.inchesToMeters(559.34), Units.inchesToMeters(79.16), new Rotation2d());

    public static final Pose2d kBlueTopAirMail =
        new Pose2d(Units.inchesToMeters(90.78), Units.inchesToMeters(237.48), new Rotation2d());
    public static final Pose2d kBlueLowAirMail =
        new Pose2d(Units.inchesToMeters(90.78), Units.inchesToMeters(79.16), new Rotation2d());
  }
}
