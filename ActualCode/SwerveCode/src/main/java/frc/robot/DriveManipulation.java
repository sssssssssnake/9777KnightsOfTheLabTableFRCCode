package frc.robot;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.TurnMotor;

public class DriveManipulation {
    private double[] swerveModuleAngles = {0, 0, 0, 0}; // in radians
    private double[] swerveModuleSpeeds = {0, 0, 0, 0}; // in meters per second
    // Angle for the translation of the robot/ module derived from position
    private double angleFromPosition;
    
    private double x;
    private double y;
    private double rotation;


    // These are constants that needn't be changed by any code

    // offset for where motors are in radians
    private double[] swerveAngleOffset = {4.2107776536,5.595962,6.062292,0.503146};

    /**
     * If the robot rotates, there are different angles that the robot uses to rotate.
     * In the case that the robot is to rotate, we need to use the vector that aligns itself with the robot's rotation
     * These are the default radian measure of the robot
     * @category Constant Expression */
    private double[] constantRotationAngle = {
        - Math.PI * 3 /4,
        - Math.PI / 4,
        - Math.PI * 5 / 4 ,
        - Math.PI * 7 / 4
    };

    private XboxController controller;
    private TurnMotor frontLeft  = new TurnMotor( 9, 16);
    private TurnMotor frontRight = new TurnMotor(11, 17);
    private TurnMotor backLeft   = new TurnMotor(13, 18);
    private TurnMotor backRight  = new TurnMotor(15, 19);

    private CANSparkFlex frontLeftDrive  = new CANSparkFlex( 8, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkFlex frontRightDrive = new CANSparkFlex(10, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkFlex backLeftDrive   = new CANSparkFlex(12, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkFlex backRightDrive  = new CANSparkFlex(14, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    DriveManipulation(XboxController getController) {
        controller = getController;
    }
    void setNewCenterState() {
        x        =  controller.getLeftX();
        y        = -controller.getLeftY();
        rotation =  controller.getRightX();

        if ((x < .1 && x > -.1) && (y < .1 && y > -.1) && (rotation < .1 && rotation > -.1)) {
            x = 0;
            y = 0;
            rotation = 0;
            for (int i = 0; i < 4; i++) {
                swerveModuleSpeeds[i] = 0;
            }
        } else {
            // V_p
            double[] positionVector = {x, y};
            // v_s
            double[][] newRotationVector = new double[4][2];
            double[][] constantRotationVector = new double[4][2];

            // we need to get the x and y components of the rotation vector
            for (int i = 0; i < 4; i++) {
                newRotationVector[i][0] = Math.cos(constantRotationAngle[i]);
                newRotationVector[i][1] = Math.sin(constantRotationAngle[i]);
            }

            // we then need to scale the rotation vector by the rotation value
            for (int i = 0; i < 4; i++) {
                newRotationVector[i][0] *= rotation;
                newRotationVector[i][1] *= rotation;
            }

            // now that we have the two scaled and set vector arrays, we can add them together

            // V_f
            double[][] finalVector = new double[4][2];
            for (int i = 0; i < 4; i++) {
                finalVector[i][0] = positionVector[0] + newRotationVector[i][0];
                finalVector[i][1] = positionVector[1] + newRotationVector[i][1];
            }

            // now we can get the angle from the position vector
            // for (int i = 0; i < 4; i++) {
            //     // we dont use atan2 because we want to keep the angle between 0 and 2pi
            //     if (finalVector[i][1] > 0) {
            //         swerveModuleAngles[i] = Math.acos(finalVector[i][0] / Math.sqrt(finalVector[i][0] * finalVector[i][0] + finalVector[i][1] * finalVector[i][1]));
            //     } else if (finalVector[i][1] < 0) {
            //         swerveModuleAngles[i] = Math.PI * 2 - Math.acos(finalVector[i][0] / Math.sqrt(finalVector[i][0] * finalVector[i][0] + finalVector[i][1] * finalVector[i][1]));
            //     } 
            // }

            // calculate the hypotenuse
            double hypotenuse = Math.sqrt(finalVector[0][0] * finalVector[0][0] + finalVector[0][1] * finalVector[0][1]);
            // calculate the angle using the previously commented out for loop, use hypotenuse
            for (int i = 0; i < 4; i++) {
                if (finalVector[i][1] > 0) {
                    swerveModuleAngles[i] = Math.acos(finalVector[i][0] / hypotenuse);
                } else if (finalVector[i][1] < 0) {
                    swerveModuleAngles[i] = Math.PI * 2 - Math.acos(finalVector[i][0] / hypotenuse);
                } 
            }

            // dont forget to add the offset
            for (int i = 0; i < 4; i++) {
                swerveModuleAngles[i] += swerveAngleOffset[i] + (Math.PI /2);
            }

            // now that al the angles are set, we can set the speeds
            for (int i = 0; i < 4; i++) {
                swerveModuleSpeeds[i] = Math.sqrt(finalVector[i][0] * finalVector[i][0] + finalVector[i][1] * finalVector[i][1]);
            }}
    }
    void runToState() {
        frontLeft.setDesiredAngle(swerveModuleAngles[0]);
        frontRight.setDesiredAngle(swerveModuleAngles[1]);
        backLeft.setDesiredAngle(swerveModuleAngles[2]);
        backRight.setDesiredAngle(swerveModuleAngles[3]);
    
        frontLeft.runToState();
        frontRight.runToState();
        backLeft.runToState();
        backRight.runToState();
    
        // now we can set the speeds
        frontLeftDrive.set(swerveModuleSpeeds[0]);
        frontRightDrive.set(swerveModuleSpeeds[1]);
        backLeftDrive.set(swerveModuleSpeeds[2]);
        backRightDrive.set(swerveModuleSpeeds[3]);
    }

}
