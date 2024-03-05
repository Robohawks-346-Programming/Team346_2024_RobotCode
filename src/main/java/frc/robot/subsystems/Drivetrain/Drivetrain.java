package frc.robot.subsystems.Drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {

            SwerveModule frontLeft = new SwerveModule(
                    Constants.DriveConstants.FRONT_LEFT_DRIVE_ID,
                    Constants.DriveConstants.FRONT_LEFT_TURN_ID,
                    Constants.DriveConstants.FRONT_LEFT_ENCODER_ID,
                    Constants.DriveConstants.FRONT_LEFT_TURN_OFFSET,
                    Constants.DriveConstants.FRONT_LEFT_DRIVE_MOTOR_INVERT
                );
        
            SwerveModule frontRight = new SwerveModule(
                Constants.DriveConstants.FRONT_RIGHT_DRIVE_ID,
                Constants.DriveConstants.FRONT_RIGHT_TURN_ID,
                Constants.DriveConstants.FRONT_RIGHT_ENCODER_ID,
                Constants.DriveConstants.FRONT_RIGHT_TURN_OFFSET,
                Constants.DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_INVERT
                ); 
        
            SwerveModule backLeft = new SwerveModule(
                Constants.DriveConstants.BACK_LEFT_DRIVE_ID,
                Constants.DriveConstants.BACK_LEFT_TURN_ID,
                Constants.DriveConstants.BACK_LEFT_ENCODER_ID,
                Constants.DriveConstants.BACK_LEFT_TURN_OFFSET,
                Constants.DriveConstants.BACK_LEFT_DRIVE_MOTOR_INVERT
                );
        
            SwerveModule backRight = new SwerveModule(
                Constants.DriveConstants.BACK_RIGHT_DRIVE_ID,
                Constants.DriveConstants.BACK_RIGHT_TURN_ID,
                Constants.DriveConstants.BACK_RIGHT_ENCODER_ID,
                Constants.DriveConstants.BACK_RIGHT_TURN_OFFSET,
                Constants.DriveConstants.BACK_RIGHT_DRIVE_MOTOR_INVERT
                );

    SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};

    AHRS gyro = new AHRS();

    private final double[] lockAngles = new double[] { 45, 315, 45, 315 };

    private double lastFPGATimestamp;

    private Field2d field = new Field2d();  

    public Pose3d lastPose3d;

    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveDriveOdometry odometry;

    private final Field2d mainField = new Field2d();
    private final Field2d odometryField = new Field2d();

    public Drivetrain() {
        gyro.enableBoardlevelYawReset(true);
        
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.DriveConstants.DRIVE_KINEMATICS,
            getHeading(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)) // doesn't matter
        );

        mainField.setRobotPose(new Pose2d(1.9, 4.99, Rotation2d.fromDegrees(0)));
        SmartDashboard.putData("Field Pose", mainField);
        odometry = new SwerveDriveOdometry(Constants.DriveConstants.DRIVE_KINEMATICS, getHeading(), getModulePositions());
       
        for( SwerveModule module : modules) {
            module.resetDistance();
            //module.syncTurnEncoders();
        }

        lastFPGATimestamp = Timer.getFPGATimestamp();

        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("Field").setPoses(poses));
        SmartDashboard.putData("Field", field);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Velocity Output", backLeft.getDriveVelocity());
        // SmartDashboard.putNumber("Gyro Heading", gyro.getRotation2d().getDegrees());
        // SmartDashboard.putNumber("Front Right", frontRight.canCoderRotations());
        // SmartDashboard.putNumber("Front Left", frontLeft.canCoderRotations());
        // SmartDashboard.putNumber("Back Right", backRight.canCoderRotations());
        // SmartDashboard.putNumber("Back Left", backLeft.canCoderRotations());

        poseEstimator.update(getHeading(), getModulePositions());
        odometry.update(getHeading(), getModulePositions());

        SmartDashboard.putNumber("Pose X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Pose Y", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Pose Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());

        //frontLeft.setDriveWheelsToVoltage(11);

    }

    public Rotation2d getHeading() {
        float rawYaw = gyro.getYaw();
        float calcYaw = rawYaw;
        if(0.0 > rawYaw) {
            calcYaw +=360.0;
        }
        return Rotation2d.fromDegrees(-calcYaw);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public SwerveModuleState[] getModuleState() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int i=0; i<=3; i++) {
            states[i] = modules[i].getState(); 
        }

        return states;
    }

    public void setFieldToVehicle(Pose2d fieldToVehicle) {
        poseEstimator.resetPosition(getHeading(), getModulePositions(), fieldToVehicle);
        odometry.resetPosition(getHeading(), getModulePositions(), fieldToVehicle);
    }

    public void brake() {
        for (int i = 0; i < modules.length; i++) {
          modules[i].setState(new SwerveModuleState(0, Rotation2d.fromDegrees(lockAngles[i])));
        }
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = Constants.DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DriveConstants.MAX_MOVE_VELOCITY_FAST);

        setModuleStates(moduleStates);
    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        for (int i = 0; i < moduleStates.length; i++) {
          modules[i].setState(moduleStates[i]);
        }
    }  

    public void resetEncoders() {
        for (SwerveModule module : modules) {
            module.resetToAbsolute();
        }
    }

    public void zeroHeading() {
        gyro.zeroYaw();
    }   

    public void resetFrontLeftAbsoluteEncoder() {
        modules[0].resetAbsoluteEncoder();
    }

    public void resetFrontRightAbsoluteEncoder() {
        modules[1].resetAbsoluteEncoder();
    }

    public void resetBackLeftAbsoluteEncoder() {
        modules[2].resetAbsoluteEncoder();
    }

    public void resetBackRightAbsoluteEncoder() {
        modules[3].resetAbsoluteEncoder();
    }

      public double getFrontLeftMetersDriven() {
        return Math.abs(frontLeft.getMetersDriven());
    }

    public void resetFrontLeftDistance() {
    frontLeft.resetDistance();
    }

    public double getPitchDegrees(){
        return gyro.getPitch();
    }

    public double getPitchRadians(){
        return Units.degreesToRadians(getPitchDegrees());
    }

    public ChassisSpeeds getSpeeds() {
        return Constants.DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleState());
      }

      public Pose2d getPose() {
        return odometry.getPoseMeters();
      }

      public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
        odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    public double getDistanceFromSpeaker() {
        if (DriverStation.getAlliance().get() == Alliance.Blue){
            return poseEstimator.getEstimatedPosition().getX();
        } else {
            return 16.5 - poseEstimator.getEstimatedPosition().getX();
        }
    }

    public void tuneFeedForward(double voltage){
        for (SwerveModule module: modules){
            module.setDriveWheelsToVoltage(voltage);
        }
    }
    public Pose2d automaticRotation(Pose2d currentPose2d){
        double x;
        double y = currentPose2d.getY()-5.5;
        if (DriverStation.getAlliance().get() == Alliance.Blue){
              x = currentPose2d.getX()-0.5;
        } else {
              x = currentPose2d.getX()-16;
        }
        return new Pose2d(currentPose2d.getX(),currentPose2d.getY(), new Rotation2d(Math.atan2(y,x)));
    }
 


}