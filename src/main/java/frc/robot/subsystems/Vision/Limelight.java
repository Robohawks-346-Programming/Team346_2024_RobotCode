package frc.robot.subsystems.Vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Limelight {
    NetworkTable limeLightTable = NetworkTableInstance.getDefault().getTable("/limelight");
    private boolean seesNote = false;
    double forwardDistance, sideDistance;

    double x, y;

    public void periodic() {
        if (limeLightTable.getEntry("tv").getDouble(0) == 1){
            seesNote = true;
            forwardDistance = Units.inchesToMeters(11.5) * 
            Math.tan(Units.degreesToRadians(10 + limeLightTable.getEntry("ty").getDouble(0)));
            sideDistance = forwardDistance / Math.tan(Units.degreesToRadians(limeLightTable.getEntry("tx").getDouble(0)));
        } else {
            seesNote = false;
            forwardDistance = 0;
            sideDistance = 0;

        }
    }

    public boolean getSeesNote() {
        return seesNote;
    }

    public double getForwardDistance(){
        return forwardDistance;
    }

    public double getSideDistance(){
        return sideDistance;
    }
}
