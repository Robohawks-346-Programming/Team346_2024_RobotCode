package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

    private final Arducam[] cameras;

    private final Notifier notifier;

    public Vision() {

        cameras = new Arducam[] {
            new Arducam(VisionConstants.cameraNames[0], VisionConstants.vehicleToCameras[0]),
            new Arducam(VisionConstants.cameraNames[1], VisionConstants.vehicleToCameras[1]),
            new Arducam(VisionConstants.cameraNames[2], VisionConstants.vehicleToCameras[2]),
            new Arducam(VisionConstants.cameraNames[3], VisionConstants.vehicleToCameras[3])
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

        RobotState.getInstance().getFieldToVehicle();

        for (int i = 0; i < 3; i++) {
            if (cameras[i].hasNewObservation()) {
                cameras[i].recordVisionObservation();
            }
        }

    }

}