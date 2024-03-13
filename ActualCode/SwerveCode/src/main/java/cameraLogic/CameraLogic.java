package cameraLogic;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public final class CameraLogic {
    static NetworkTable camera = NetworkTableInstance.getDefault().getTable("limelight");
    static double[][] rotateVector = {
        {1,0,0},
        {0, Math.sqrt(3)/2, 1/2},
        {0, -1/2, Math.sqrt(3)/2},
    };

    static double[] updatedTargetPos = new double[6];

    public static void updateValues(){
        NetworkTableEntry pose = camera.getEntry("targetpose_robotspace");
        updatedTargetPos = pose.getDoubleArray(new double[6]);
    }

    public static double[] postXYZ() {
        updateValues();
        double[] postXYZ = new double[3];
        for (int i = 0; i < 3; i++) {
            postXYZ[i] = updatedTargetPos[i];
        }
        return postXYZ;
    }
    





    
}
