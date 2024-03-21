package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

    private final Arducam[] cameras;
    private double frCount;
    private double brCount;
    private double blCount;
    private double flCount;

    private final Notifier notifier;

    public Vision() {

        cameras = new Arducam[] {
            new Arducam(Constants.VisionConstants.cameraNames[0], VisionConstants.vehicleToCameras[0], flCount),
            new Arducam(Constants.VisionConstants.cameraNames[1], VisionConstants.vehicleToCameras[1], frCount),
            new Arducam(Constants.VisionConstants.cameraNames[2], VisionConstants.vehicleToCameras[2], blCount),
            new Arducam(Constants.VisionConstants.cameraNames[3], VisionConstants.vehicleToCameras[3], brCount)
        };

        notifier = new Notifier(() -> {
            for (int i = 0; i < 3; i++) {
                cameras[i].periodic();
            }
        });
        notifier.startPeriodic(0.02);

    }

    @Override
    public void periodic() {

        RobotContainer.drivetrain.poseEstimator.getEstimatedPosition();
        SmartDashboard.putNumber("FL", flCount);
        SmartDashboard.putNumber("BL", blCount);
        SmartDashboard.putNumber("FR", frCount);
        SmartDashboard.putNumber("BR", brCount);

        for (int i = 0; i < 3; i++) {
            if (cameras[i].hasNewObservation()) {
                cameras[i].recordVisionObservation();
            }
        }

    }

}