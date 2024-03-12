package frc.robot.teleopSwerve;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HardThenSoft;

public class DriveManipulation {
    private double[] swerveModuleAngles = {0, 0, 0, 0}; // in radians
    private double[] swerveModuleSpeeds = {0, 0, 0, 0}; // in meters per second
    // Angle for the translation of the robot/ module derived from position
    private double angleFromPosition;
    
    private double x;
    private double y;
    private double rotation;

    boolean brakeRotation = false;

    // These are constants that needn't be changed by any code

    // offset for where motors are in radians
    private double[] swerveAngleOffset = {Math.PI / 2,- Math.PI / 4, Math.PI / 2 , 3 * Math.PI / 4};
    
    /**
     * If the robot rotates, there are different angles that t he robot uses to rotate.
     * In the case that the robot is to rotate, we need to use the vector that aligns itself with the robot's rotation
     * These are the default radian measure of the robot
     * @category Constant Expression */
    private double[] constantRotationAngle = {
          Math.PI / 4,
        - Math.PI / 4,
          Math.PI * 3 / 4,
        - Math.PI * 3 / 4,
    };

    public XboxController controller;
    private TurnMotor frontLeft  = HardThenSoft.frontLeft;
    private TurnMotor frontRight = HardThenSoft.frontRight;
    private TurnMotor backLeft   = HardThenSoft.backLeft;
    private TurnMotor backRight  = HardThenSoft.backRight;

    private CANSparkFlex frontLeftDrive  = HardThenSoft.frontLeftDrive;
    private CANSparkFlex frontRightDrive = HardThenSoft.frontRightDrive;
    private CANSparkFlex backLeftDrive   = HardThenSoft.backLeftDrive;
    private CANSparkFlex backRightDrive  = HardThenSoft.backRightDrive;

    public DriveManipulation(XboxController getController) {
        controller = getController;
        swerveAngleOffset[0] += Math.PI / 4;
        swerveAngleOffset[0] += 25 * Math.PI / 180;
        swerveAngleOffset[1] -= 22 * Math.PI / 180;
        swerveAngleOffset[2] += Math.PI / 8;
        swerveAngleOffset[2] += 15 * Math.PI / 180;
        swerveAngleOffset[3] -=  8 * Math.PI / 180;
    }
    public void setNewCenterState() {
        x        =  controller.getLeftX();
        y        = -controller.getLeftY();
        rotation =  controller.getRightX();

        double oldX = x;
        double oldY = y;

        double angleFromNavX = HardThenSoft.navx.getAngle() / 180 * Math.PI;
        angleFromNavX += Math.PI + HardThenSoft.gyroOffset;
        x = oldX * Math.cos(angleFromNavX) - oldY * Math.sin(angleFromNavX);
        y = oldX * Math.sin(angleFromNavX) + oldY * Math.cos(angleFromNavX);

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
            
            if (rotation < .1 && rotation > -.1) {
                brakeRotation = false;
            } else {
                if (Math.sqrt(positionVector[0] * positionVector[0] + positionVector[1] * positionVector[1]) < .1) {
                    brakeRotation = true;
                } else {
                    brakeRotation = false;
                }
            }

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

            SmartDashboard.putNumber("Front Left Angle", swerveModuleAngles[0]);
            SmartDashboard.putNumber("Front Right Angle", swerveModuleAngles[1]);
            SmartDashboard.putNumber("Back Left Angle", swerveModuleAngles[2]);
            SmartDashboard.putNumber("Back Right Angle", swerveModuleAngles[3]);

            // dont forget to add the offset
            for (int i = 0; i < 4; i++) {
                swerveModuleAngles[i] += swerveAngleOffset[i]  + (Math.PI /2);
            }

            // now that al the angles are set, we can set the speeds
            for (int i = 0; i < 4; i++) {
                swerveModuleSpeeds[i] = Math.sqrt(finalVector[i][0] * finalVector[i][0] + finalVector[i][1] * finalVector[i][1]);
            }}
    }
    public void runToState(boolean precisionMode) {
        frontLeft.setDesiredAngle(swerveModuleAngles[0]);
        frontRight.setDesiredAngle(swerveModuleAngles[1]);
        backLeft.setDesiredAngle(swerveModuleAngles[2]);
        backRight.setDesiredAngle(swerveModuleAngles[3]);
    
        frontLeft.runToState();
        frontRight.runToState();
        backLeft.runToState();
        backRight.runToState();
    
        double sum = 0;
        double avg = 0;
        
        
        // now we can set the speeds

        //average the speeds to each of the motors
        for(int i = 0; i < swerveModuleSpeeds.length; i++){  
            //getting elements from the list and adding to the variable sum   
            sum = sum + swerveModuleSpeeds[i];  
            //finds the average of the list  
            avg = sum / swerveModuleSpeeds.length;   

        }

        if (brakeRotation) {
            frontLeftDrive.setIdleMode(CANSparkFlex.IdleMode.kBrake);
            frontRightDrive.setIdleMode(CANSparkFlex.IdleMode.kBrake);
            backLeftDrive.setIdleMode(CANSparkFlex.IdleMode.kBrake);
            backRightDrive.setIdleMode(CANSparkFlex.IdleMode.kBrake);
        } else {
            frontLeftDrive.setIdleMode(CANSparkFlex.IdleMode.kCoast);
            frontRightDrive.setIdleMode(CANSparkFlex.IdleMode.kCoast);
            backLeftDrive.setIdleMode(CANSparkFlex.IdleMode.kCoast);
            backRightDrive.setIdleMode(CANSparkFlex.IdleMode.kCoast);
        }


        //Apply power deadband to keep motors from coasting
        if(Math.abs(avg) < .1){
            frontLeftDrive.stopMotor();
            frontRightDrive.stopMotor();
            backLeftDrive.stopMotor();
            backRightDrive.stopMotor();
        }else{
            if(precisionMode){
                frontLeftDrive.set (swerveModuleSpeeds[0] * .2);
                frontRightDrive.set(swerveModuleSpeeds[1] * .2);
                backLeftDrive.set  (swerveModuleSpeeds[2] * .2);
                backRightDrive.set (swerveModuleSpeeds[3] * .2);


            }else{
                frontLeftDrive.set (swerveModuleSpeeds[0] * .7);
                frontRightDrive.set(swerveModuleSpeeds[1] * .7);
                backLeftDrive.set  (swerveModuleSpeeds[2] * .7);
                backRightDrive.set (swerveModuleSpeeds[3] * .7);

                frontLeftDrive.setIdleMode(CANSparkFlex.IdleMode.kCoast);
                frontRightDrive.setIdleMode(CANSparkFlex.IdleMode.kCoast);
                backLeftDrive.setIdleMode(CANSparkFlex.IdleMode.kCoast);
                backRightDrive.setIdleMode(CANSparkFlex.IdleMode.kCoast);

            }

        }


    }

}
