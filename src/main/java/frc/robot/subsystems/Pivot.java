package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Pivot extends SubsystemBase {
    private static TalonFX pivotMotor;
    private static TalonFXConfiguration pivotMotorConfig;
    
   private PositionVoltage position;

   public final InterpolatingDoubleTreeMap pivotLookupTable = Constants.PivotConstants.getPivotMap();

       private ArmFeedforward feedforward =
            new ArmFeedforward(
                    Constants.PivotConstants.PIVOT_kS,
                    Constants.PivotConstants.PIVOT_kG,
                    Constants.PivotConstants.PIVOT_kV,
                    Constants.PivotConstants.PIVOT_kA);
    
    public Pivot() {
        pivotMotor = new TalonFX(Constants.PivotConstants.PIVOT_MOTOR_ID);
        pivotMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        pivotMotor.setInverted(true);

        pivotMotorConfig.Slot0.kP = Constants.PivotConstants.PIVOT_P;
        pivotMotorConfig.Slot0.kI = Constants.PivotConstants.PIVOT_I;
        pivotMotorConfig.Slot0.kD = Constants.PivotConstants.PIVOT_D;
        pivotMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotMotorConfig.Slot0.kG = Constants.PivotConstants.PIVOT_kG;
        pivotMotorConfig.Slot0.kA = Constants.PivotConstants.PIVOT_kA;
        pivotMotorConfig.Slot0.kV = Constants.PivotConstants.PIVOT_kV;
        pivotMotorConfig.Slot0.kS = Constants.PivotConstants.PIVOT_kS;

        pivotMotorConfig.Feedback.SensorToMechanismRatio = Constants.PivotConstants.PIVOT_GEAR_RATIO;

        pivotMotor.getConfigurator().apply(pivotMotorConfig);

        position = new PositionVoltage(0);

        // pivotMotor.setPosition(convertDegreesToRotations(Constants.PivotConstants.HOME_PIVOT_ANGLE));
        resetPivotAngle();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Degrees", convertRotationsToDegrees(pivotMotor.getPosition().getValue()));
    }
    
    public boolean isAtPosition() {
        return(Math.abs(pivotLookupTable.get(RobotContainer.drivetrain.getDistanceFromSpeaker()) - convertRotationsToDegrees(pivotMotor.getPosition().getValueAsDouble())) < 2);
    }

    public void moveArmToPosition(double wantedPosition) {
        pivotMotor.setControl(position.withPosition(convertDegreesToRotations(wantedPosition)));
        //pivotMotor.setVoltage(feedforward.calculate(Math.toRadians(wantedPosition), 0));
    }
    public double convertDegreesToRotations(double degrees){
        return (degrees / 360.0);
    }

    public double convertRotationsToDegrees(double rotations){
        return (rotations * 360.0);
    }

    public Command moveArm(double pos) {
        return Commands.runOnce(() -> pivotMotor.setControl(position.withPosition(convertDegreesToRotations(pos))));
    }
    
    public Command distanceBasedArmPivot(){
        // SmartDashboard.putNumber("Distance from Speaker", RobotContainer.drivetrain.getDistanceFromSpeaker());
        // SmartDashboard.putNumber("Wanted Arm Angle", pivotLookupTable.get(RobotContainer.drivetrain.getDistanceFromSpeaker()));
        return Commands.runOnce(() -> pivotMotor.setControl(position.withPosition(convertDegreesToRotations(pivotLookupTable.get(RobotContainer.drivetrain.getDistanceFromSpeaker())))));
    }
    
    public void resetPivotAngle() {
        pivotMotor.setPosition(convertDegreesToRotations(Constants.PivotConstants.HOME_PIVOT_ANGLE));
    }

    public void stopPivot() {
        pivotMotor.stopMotor();
    }

    public double lerp(double a, double b, double c, double d){
        return (c * (b - pivotMotor.getPosition().getValue()) + d * (pivotMotor.getPosition().getValue() - a)) / (b - a);
    }

    public void moveArmLerp(double wantedRevs, double curretRevs){
        double currentPose = getPosition();
        if (wantedRevs > currentPose){
            pivotMotor.set(lerp(curretRevs, wantedRevs, 1, 0.3));
        } else if (wantedRevs < currentPose){
            pivotMotor.set(-lerp(curretRevs, wantedRevs, 1, 0.3));
        } else {
           pivotMotor.set(0);
        }
    }
    
    public double getPosition() {
        return pivotMotor.getPosition().getValue();
    }

    public double getDegrees(){
        return convertRotationsToDegrees(pivotMotor.getPosition().getValue());
    }

    public void setPercent(){
        pivotMotor.set(-0.06);
    }

    public void driveDown(){
        pivotMotor.set(-0.02);
    }
}