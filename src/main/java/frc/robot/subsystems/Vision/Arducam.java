package frc.robot.subsystems.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.VisionConstants;

public class Arducam {

    private static final AprilTagFieldLayout tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private volatile boolean hasNewPose = false;
    private volatile Pose3d calculatedPose = new Pose3d();
    private volatile Matrix<N3, N1> stdDevs = VecBuilder.fill(1000, 1000, 1000);
    private volatile double timestamp = 1;

    private String name;


    public Arducam(String cameraName, Transform3d vehicleToCamera) {
        camera = new PhotonCamera(cameraName);
        poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, vehicleToCamera);
        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

        name = cameraName;
    }

    public void periodic() {

        if (!camera.isConnected()) return;

        PhotonPipelineResult result = camera.getLatestResult(); 
        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
        if (estimatedPose.isEmpty()) return;
        EstimatedRobotPose estimation = estimatedPose.get();
        if (estimation.timestampSeconds == timestamp) return;
        if (estimation.targetsUsed.size() == 1 && 
            (estimation.targetsUsed.get(0).getPoseAmbiguity() > Constants.VisionConstants.SINGLE_TAG_AMBIGUITY_CUTOFF || estimation.targetsUsed.get(0).getPoseAmbiguity() == -1))
            return;

        double distance = 0;
        for (PhotonTrackedTarget target : estimation.targetsUsed) {
            distance += target.getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
        }

        distance /= estimation.targetsUsed.size();
        stdDevs = computeStdDevs(distance);

        calculatedPose = estimation.estimatedPose;
        timestamp = estimation.timestampSeconds;
        hasNewPose = true;

    }

    public boolean hasNewObservation() {
        return hasNewPose;
    }

    public void recordVisionObservation() {
        RobotState.getInstance().recordVisionObservations(calculatedPose.toPose2d(), stdDevs, timestamp);
        hasNewPose = false;
    }

    private Matrix<N3, N1> computeStdDevs(double distance) {
        double stdDev = Math.max(
            Constants.VisionConstants.MINIMUM_STANDARD_DEVIATION, 
            Constants.VisionConstants.EULER_MULTIPLIER * Math.exp(distance * Constants.VisionConstants.DISTANCE_MULTIPLIER)
        );
        return VecBuilder.fill(stdDev, stdDev, 1000);
    }
    
}