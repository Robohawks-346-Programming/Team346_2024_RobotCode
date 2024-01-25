package frc.robot.subsystems.Drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;

public class Drivetrain extends SubsystemBase {


            SwerveModule frontLeft = new SwerveModule(
                    Constants.DriveConstants.FRONT_LEFT_DRIVE_ID,
                    Constants.DriveConstants.FRONT_LEFT_TURN_ID,
                    Constants.DriveConstants.FRONT_LEFT_ENCODER_ID,
                    Constants.DriveConstants.FRONT_LEFT_TURN_OFFSET
                );
        
            SwerveModule frontRight = new SwerveModule(
                Constants.DriveConstants.FRONT_RIGHT_DRIVE_ID,
                Constants.DriveConstants.FRONT_RIGHT_TURN_ID,
                Constants.DriveConstants.FRONT_RIGHT_ENCODER_ID,
                Constants.DriveConstants.FRONT_RIGHT_TURN_OFFSET
                ); 
        
            SwerveModule backLeft = new SwerveModule(
                Constants.DriveConstants.BACK_LEFT_DRIVE_ID,
                Constants.DriveConstants.BACK_LEFT_TURN_ID,
                Constants.DriveConstants.BACK_LEFT_ENCODER_ID,
                Constants.DriveConstants.BACK_LEFT_TURN_OFFSET
                );
        
            SwerveModule backRight = new SwerveModule(
                Constants.DriveConstants.BACK_RIGHT_DRIVE_ID,
                Constants.DriveConstants.BACK_RIGHT_TURN_ID,
                Constants.DriveConstants.BACK_RIGHT_ENCODER_ID,
                Constants.DriveConstants.BACK_RIGHT_TURN_OFFSET
                );

    SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};

    AHRS gyro = new AHRS();

    private final double[] lockAngles = new double[] { 45, 315, 45, 315 };

    private double lastFPGATimestamp;

    private Field2d field = new Field2d();  

    public Pose3d lastPose3d;

    public Drivetrain() {
        gyro.enableBoardlevelYawReset(true);
        
        RobotState.getInstance().initPoseEstimator(getHeading(), getModulePositions());
       
        for( SwerveModule module : modules) {
            module.resetDistance();
            //module.syncTurnEncoders();
        }

        lastFPGATimestamp = Timer.getFPGATimestamp();

        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
        SmartDashboard.putData("Field", field);
    }
    
    @Override
    public void periodic() {
        // if (lastFPGATimestamp < Timer.getFPGATimestamp()) {
        //     lastFPGATimestamp = Timer.getFPGATimestamp() + 1;
        //     for (SwerveModule module : modules) {
        //         module.syncTurnEncoders();
        //     }
        // }

        SmartDashboard.putBoolean("Gyro calibration Status", gyro.isCalibrating());
        SmartDashboard.putNumber("Velocity Output", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("Gyro Heading", gyro.getRotation2d().getDegrees());
        SmartDashboard.putNumber("Front Right", frontRight.turnAngleDegrees());
        SmartDashboard.putNumber("Front Left", frontLeft.turnAngleDegrees());
        SmartDashboard.putNumber("Back Right", backRight.turnAngleDegrees());
        SmartDashboard.putNumber("Back Left", backLeft.turnAngleDegrees());
        SmartDashboard.putNumber("Pitch", getPitchDegrees());

        RobotState.getInstance().recordDriveObservations(getHeading(), getModulePositions());

    }

    public Rotation2d getHeading() {
        float rawYaw = gyro.getYaw();
        float calcYaw = rawYaw;
        if(0.0 > rawYaw) {
            calcYaw +=360.0;
        }
        return Rotation2d.fromDegrees(-calcYaw);
    }

    // public SwerveModulePosition[] getModulePositions() {
    //     SwerveModulePosition[] position = new SwerveModulePosition[4];

    //     for (int i2=0; i2<=3; i2++) {
    //         position[i2++] = modules[i2].getPosition(); 
    //     }

    //     return position;
    // }

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
            states[i++] = modules[i].getState(); 
        }

        return states;
    }

    public void setFieldToVehicle(Pose2d fieldToVehicle) {
        RobotState.getInstance().setFieldToVehicle(getHeading(), getModulePositions(), fieldToVehicle);
    }

    public void brake() {
        for (int i = 0; i < modules.length; i++) {
          modules[i].setState(new SwerveModuleState(0, Rotation2d.fromDegrees(lockAngles[i])));
        }
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = Constants.DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);

        for (int i = 0; i < moduleStates.length; i++) {
            moduleStates[i] = SwerveModuleState.optimize(moduleStates[i], modules[i].getState().angle);
        }

        setModuleStates(moduleStates);
    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        for (int i = 0; i < moduleStates.length; i++) {
          modules[i].setState(moduleStates[i]);
        }
    }  

    public void resetEncoders() {
        for (SwerveModule module : modules) {
            module.resetEncoders();
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
        return RobotState.getInstance().getFieldToVehicle();
      }

      public void resetPose(Pose2d pose) {
        RobotState.getInstance().setFieldToVehicle(gyro.getRotation2d(), getModulePositions(), pose);
    }
}