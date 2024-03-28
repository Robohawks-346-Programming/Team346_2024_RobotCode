package frc.robot.subsystems.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;

public class Arducam {

    private static final AprilTagFieldLayout tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private volatile boolean hasNewPose = false;
    private volatile Pose3d calculatedPose = new Pose3d();
    private volatile Pose3d intermediatePose = new Pose3d();
    private volatile double timestamp = 1;
    private volatile Matrix<N3, N1> stdDevs = VecBuilder.fill(1000, 1000, 1000);
    private String name;



    public Arducam(String cameraName, Transform3d vehicleToCamera) {
        camera = new PhotonCamera(cameraName);
        poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, vehicleToCamera);
        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        name = cameraName;
    }

    public void periodic() {
        SmartDashboard.putBoolean(name, camera.isConnected());
       // SmartDashboard.putNumber(name, count);
        if (!camera.isConnected()) return;

        PhotonPipelineResult result = camera.getLatestResult();
        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
        if (estimatedPose.isEmpty()) return;
        EstimatedRobotPose estimation = estimatedPose.get();
        if (estimation.timestampSeconds == timestamp) return;
        if (estimation.targetsUsed.size() == 1 && 
            (estimation.targetsUsed.get(0).getPoseAmbiguity() > Constants.VisionConstants.SINGLE_TAG_AMBIGUITY_CUTOFF || estimation.targetsUsed.get(0).getPoseAmbiguity() == -1))
            return;

        intermediatePose = estimation.estimatedPose;

        if (intermediatePose.getZ() > 1 || intermediatePose.getZ() < -0.1) {
            return;
        }

        if (Math.abs(Math.toDegrees(intermediatePose.getRotation().getAngle()) - RobotContainer.drivetrain.poseEstimator.getEstimatedPosition().getRotation().getDegrees()) < 2){
            return;
        }

        if (Math.abs(Math.toDegrees(intermediatePose.getX()) - RobotContainer.drivetrain.poseEstimator.getEstimatedPosition().getX()) < 0.025){
            return;
        }

        if (Math.abs(Math.toDegrees(intermediatePose.getY()) - RobotContainer.drivetrain.poseEstimator.getEstimatedPosition().getY()) < 0.025){
            return;
        }

        calculatedPose = estimation.estimatedPose;
        timestamp = estimation.timestampSeconds;
        hasNewPose = true;

        double distance = 0;
        for (PhotonTrackedTarget target : estimation.targetsUsed) {
            distance += target.getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
        }

        distance /= estimation.targetsUsed.size();
        stdDevs = computeStdDevs(distance);
        SmartDashboard.putBoolean(name+"Works", true);

    }

    public boolean hasNewObservation() {
        return hasNewPose;
    }

    public void recordVisionObservation() {
        RobotContainer.drivetrain.poseEstimator.addVisionMeasurement(calculatedPose.toPose2d(), timestamp, stdDevs);
        hasNewPose = false;
    }

    private Matrix<N3, N1> computeStdDevs(double distance) {
        double stdDev = Math.max(
            VisionConstants.minimumStdDev, 
            VisionConstants.stdDevEulerMultiplier * Math.exp(distance * VisionConstants.stdDevDistanceMultiplier)
        );
        return VecBuilder.fill(stdDev, stdDev, 1000);
    }
    
}