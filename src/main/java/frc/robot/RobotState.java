package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

public class RobotState {

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }

    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveDriveOdometry odometry;

    public void initPoseEstimator(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.DriveConstants.DRIVE_KINEMATICS,
            rotation,
            modulePositions,
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)) // doesn't matter
        );
        odometry = new SwerveDriveOdometry(Constants.DriveConstants.DRIVE_KINEMATICS, rotation, modulePositions);
    }

    public void setFieldToVehicle(Rotation2d rotation, SwerveModulePosition[] modulePositions, Pose2d fieldToVehicle) {
        poseEstimator.resetPosition(rotation, modulePositions, fieldToVehicle);
        odometry.resetPosition(rotation, modulePositions, fieldToVehicle);
    }

    public void recordDriveObservations(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(rotation, modulePositions);
        odometry.update(rotation, modulePositions);

        SmartDashboard.putNumber("Odometry X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Odometry Y", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Odometry Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    }

    public void recordVisionObservations(Pose2d pose, Matrix<N3, N1> stdDevs, double timestamp) {
        
        poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);

    }

    public Pose2d getFieldToVehicle() {       
        return poseEstimator.getEstimatedPosition();    
    }

    public Pose2d getOdometryFieldToVehicle() {
        return odometry.getPoseMeters();
    }
}
