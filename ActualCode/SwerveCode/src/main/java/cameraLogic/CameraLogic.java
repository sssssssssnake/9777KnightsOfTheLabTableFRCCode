package cameraLogic;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomousCommands.PositionThread;


public final class CameraLogic {
    static NetworkTable camera = NetworkTableInstance.getDefault().getTable("limelight");
    static double[][] rotateVector = {
        {1,0,0},
        {0, Math.sqrt(3)/2, 1/2},
        {0, -1/2, Math.sqrt(3)/2},
    };

    static double[] updatedTargetPos = new double[6];

    static final int[] tagIds = {
        7,
        4,
        11,
        13,
        14,
        15,
        16
    };

    public static double[][] idTarget = {
        //Distance from Speaker Tag
        {0, 1.9, 0},
        //distance from hang
        {-.4, 48.2, 0},
    };

    

 

    public static void autoAlign(){
        NetworkTableEntry id = camera.getEntry("tid");
        updateValues();
        if (updatedTargetPos[0] == 0.0) {
            return;
        }
        double[] targetPose = new double[3];
        Thread run = null;
        int iid = (int) id.getDouble(0);
        if (iid == 0) {
            return;
        }

        if (iid == tagIds[0] || iid == tagIds[1]) {
            targetPose[0] = (updatedTargetPos[0] - idTarget[0][0]) * 100;
            targetPose[1] = (updatedTargetPos[2] - idTarget[0][1]) * 100;
            targetPose[2] = updatedTargetPos[4] * Math.PI/180;
            moveTo(targetPose);
        } else if (iid == tagIds[2] || iid == tagIds[3] || iid == tagIds[4] || iid == tagIds[5] || iid == tagIds[6]) {
            targetPose[0] = (updatedTargetPos[0] - idTarget[1][0]) * 100;
            targetPose[1] = (updatedTargetPos[2] - idTarget[1][1]) * 100;
            targetPose[2] = updatedTargetPos[4] * Math.PI/180;
            moveTo(targetPose);
        }
    }
    
    public static void updateValues(){
        NetworkTableEntry pose = camera.getEntry("targetpose_robotspace");
        updatedTargetPos = pose.getDoubleArray(new double[6]);
    }

    public static double[] postXYZ() {
        updateValues();
        double[] postXYZ = new double[3];
        postXYZ[0] = (updatedTargetPos[0] - idTarget[0][0]) * 100;
        postXYZ[1] = (updatedTargetPos[2] - idTarget[0][1]) * 100;
        postXYZ[2] = updatedTargetPos[4] * Math.PI/180;

        SmartDashboard.putNumber("ID", camera.getEntry("tid").getDouble(0));
        return postXYZ;
    } 

    // make a method that takes a double[] and moves the robot to that position
    public static void moveTo(double[] targetPose) {
        Thread run = new Thread(new PositionThread(targetPose[0], targetPose[1], targetPose[2]));
        run.start();
    }

    
}
