package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html
// READ THIS ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

public class Vision extends SubsystemBase {

  // make sure the name in quotes is EXACTLY the same as it is in PV
  PhotonCamera FrontTagCam = new PhotonCamera("FrontTagCam");
  PhotonCamera SideTagCam = new PhotonCamera("SideTagCam");
  PhotonCamera OBJCam = new PhotonCamera("OBJCam");

  private final EstimateConsumer estConsumer;
  private final PhotonPoseEstimator frontPhotonEstimator;
  private final PhotonPoseEstimator sidePhotonEstimator;
  private Matrix<N3, N1> frontStdDevs;
  private Matrix<N3, N1> sideStdDevs;

  public Vision(EstimateConsumer estConsumer) {
    this.estConsumer = estConsumer;
    frontPhotonEstimator =
        new PhotonPoseEstimator(VisionConstants.kTagLayout, VisionConstants.kFrontRobotToCam);

    sidePhotonEstimator =
        new PhotonPoseEstimator(VisionConstants.kTagLayout, VisionConstants.kSideRobotToCam);
    frontStdDevs = VisionConstants.kFrontSingleTagStdDevs;
    sideStdDevs = VisionConstants.kSideSingleTagStdDevs;
  }

  double objYaw = 0.0;

  @Override
  public void periodic() {

    // Estimate Pose from front camera
    Optional<EstimatedRobotPose> frontEst;
    for (PhotonPipelineResult result : FrontTagCam.getAllUnreadResults()) {
      frontEst = frontPhotonEstimator.estimateCoprocMultiTagPose(result);

      if (frontEst.isEmpty()) {
        frontEst = frontPhotonEstimator.estimateLowestAmbiguityPose(result);
      }

      updateEstimationStdDevs(frontEst, result.getTargets(), true);

      frontEst.ifPresent(
          est -> {
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = getEstimationStdDevs(true);
            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
          });
    }

    // Estimate Pose from side camera
    Optional<EstimatedRobotPose> sideEst;
    for (PhotonPipelineResult result : SideTagCam.getAllUnreadResults()) {
      sideEst = sidePhotonEstimator.estimateCoprocMultiTagPose(result);

      if (sideEst.isEmpty()) {
        sideEst = sidePhotonEstimator.estimateLowestAmbiguityPose(result);
      }

      updateEstimationStdDevs(sideEst, result.getTargets(), false);

      sideEst.ifPresent(
          est -> {
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = getEstimationStdDevs(false);
            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
          });
    }

    // Object Detection
    if (OBJCam.getLatestResult().hasTargets()) {
      PhotonTrackedTarget target = OBJCam.getLatestResult().getBestTarget();
      objYaw = target.getYaw();
    } else {
      objYaw = 0.0;
    }
    Logger.recordOutput("Vision/OBJYaw", objYaw, "degrees");
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation
    // Mostly used for debug and such
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   * @param isFront Whether to calculate based off the front or side camera
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose,
      List<PhotonTrackedTarget> targets,
      boolean isFront) {

    var estimator = isFront ? frontPhotonEstimator : sidePhotonEstimator;

    Matrix<N3, N1> singleTagStdDevs =
        isFront ? VisionConstants.kFrontSingleTagStdDevs : VisionConstants.kSideSingleTagStdDevs;

    Matrix<N3, N1> multiTagStdDevs =
        isFront ? VisionConstants.kFrontMultiTagStdDevs : VisionConstants.kSideMultiTagStdDevs;

    Matrix<N3, N1> resultStdDevs;

    if (estimatedPose.isEmpty()) {
      resultStdDevs = singleTagStdDevs;
    } else {
      var pose2d = estimatedPose.get().estimatedPose.toPose2d();

      int numTags = 0;
      double avgDist = 0;

      for (PhotonTrackedTarget tgt : targets) {
        var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;

        numTags++;
        avgDist += tagPose.get().toPose2d().getTranslation().getDistance(pose2d.getTranslation());
      }

      if (numTags == 0) {
        resultStdDevs = singleTagStdDevs;
      } else {
        avgDist /= numTags;

        resultStdDevs = (numTags > 1) ? multiTagStdDevs : singleTagStdDevs;

        if (numTags == 1 && avgDist > 4) {
          resultStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          resultStdDevs = resultStdDevs.times(1 + (avgDist * avgDist / 30));
        }
      }
    }
    if (isFront) {
      frontStdDevs = resultStdDevs;
    } else {
      sideStdDevs = resultStdDevs;
    }
  }

  private Matrix<N3, N1> getEstimationStdDevs(boolean isFront) {
    return (isFront) ? frontStdDevs : sideStdDevs;
  }

  // Sends Pose to Drive
  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }

  public double getOBJYaw() {
    return objYaw;
  }
}
