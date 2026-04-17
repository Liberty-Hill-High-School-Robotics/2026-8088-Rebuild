package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class InZone {
  public static boolean isInAllianceZone(double poseX) {
    Pose2d hubSetpoint;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      hubSetpoint = FieldConstants.kBlueHubPose;
      if (poseX < hubSetpoint.getX()) {
        return true;
      }
    } else {
      hubSetpoint = FieldConstants.kRedHubPose;
      if (poseX > hubSetpoint.getX()) {
        return true;
      }
    }
    return false;
  }
}
