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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState {

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }

    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveDriveOdometry odometry;

    private final Field2d mainField = new Field2d();
    private final Field2d odometryField = new Field2d();

    private boolean isIntaking = false;
    private boolean hasIntaked = false;

    public void initPoseEstimator(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.DriveConstants.DRIVE_KINEMATICS,
            rotation,
            modulePositions,
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)) // doesn't matter
        );

        mainField.setRobotPose(new Pose2d(1.9, 4.99, Rotation2d.fromDegrees(0)));
        SmartDashboard.putData("Field Pose", mainField);
        odometry = new SwerveDriveOdometry(Constants.DriveConstants.DRIVE_KINEMATICS, rotation, modulePositions);
    }

    public void setFieldToVehicle(Rotation2d rotation, SwerveModulePosition[] modulePositions, Pose2d fieldToVehicle) {
        poseEstimator.resetPosition(rotation, modulePositions, fieldToVehicle);
        odometry.resetPosition(rotation, modulePositions, fieldToVehicle);
    }

    public void recordDriveObservations(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(rotation, modulePositions);
        odometry.update(rotation, modulePositions);

        SmartDashboard.putNumber("Pose X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Pose Y", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Pose Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    }

    public void recordVisionObservations(Pose2d pose, Matrix<N3, N1> stdDevs, double timestamp) {
        poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
        mainField.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    public Pose2d getFieldToVehicle() {       
        mainField.setRobotPose(poseEstimator.getEstimatedPosition());
        SmartDashboard.putData("Field Pose", mainField);

        odometryField.setRobotPose(odometry.getPoseMeters());
        SmartDashboard.putData("Odometry Pose", odometryField);
        
        return poseEstimator.getEstimatedPosition();   
    }

    public Pose2d getOdometryFieldToVehicle() {
        return odometry.getPoseMeters();
    }

    public boolean hasIntaked() {
        return hasIntaked;
    }

    public void setIntaked(boolean i) {
        hasIntaked = i;
    }
    
    public boolean isIntaking() {
        return isIntaking;
    }

    public void setIntaking(boolean i) {
        isIntaking = i;
    }
}
