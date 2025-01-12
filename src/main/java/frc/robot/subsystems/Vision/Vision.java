package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

    private final Arducam[] cameras;
    private final Limelight limelight;
    private double count;


    private final Notifier notifier;

    public Vision() {

        cameras = new Arducam[] {
            // new Arducam(Constants.VisionConstants.cameraNames[0], VisionConstants.vehicleToCameras[0]),
            // new Arducam(Constants.VisionConstants.cameraNames[1], VisionConstants.vehicleToCameras[1]),
            new Arducam(Constants.VisionConstants.cameraNames[0], VisionConstants.vehicleToCameras[2]),
            new Arducam(Constants.VisionConstants.cameraNames[3], VisionConstants.vehicleToCameras[3])
        };
        limelight = new Limelight();

        notifier = new Notifier(() -> {
            for (int i = 0; i < cameras.length; i++) {
                cameras[i].periodic();
            }
            limelight.periodic();
        });
        notifier.startPeriodic(0.02);

    }

    @Override
    public void periodic() {

        RobotContainer.drivetrain.poseEstimator.getEstimatedPosition();
        SmartDashboard.putNumber("Vision Count", count);

        for (int i = 0; i < cameras.length; i++) {
            if (cameras[i].hasNewObservation()) {
                cameras[i].recordVisionObservation();
                count++;
            }
        }

    }

    public boolean getNoteVisible(){
        return limelight.getSeesNote();
    }

    public double getNoteX(){
        return limelight.getForwardDistance();
    }

    public double getNoteY(){
        return limelight.getSideDistance();
    }

    public void resetDistances(){
        limelight.resetDistances();
    }

    public double getSpeaker() {
        return (cameras[3].getSpeakerFull() + cameras[2].getSpeakerFull()) / 2;
    }

}