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

    static double[] tagIds = {
        7,
        4,
    };

    public static double[][] idTarget = {
        //Distance from Speaker Tag
        {0, 1.9, 0}
    };
    

 

    public static void autoAlign(){
        NetworkTableEntry id = camera.getEntry("tid");
        if (updatedTargetPos[0] == 0.0) {
            return;
        }
        switch ((int)id.getDouble(0.0)) {
            //Red and Blue Speaker
            case 4:
            case 7:
                updateValues();
                double[] targetPose = {
                    (updatedTargetPos[0] - idTarget[0][0])*100,
                    (updatedTargetPos[2] - idTarget[0][1])*100,
                    updatedTargetPos[4]/180*Math.PI - idTarget[0][2],
                };

                Thread run = new Thread(new PositionThread(targetPose[0], targetPose[1], targetPose[2]));
                run.start();            
                break;
            case 0:
            default:
                break;
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
    



    
}
