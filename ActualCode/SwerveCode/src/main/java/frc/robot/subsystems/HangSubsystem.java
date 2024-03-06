package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.HardThenSoft;

public class HangSubsystem {



    //FIXME: Tune the Hang PID Values
    PIDController hangVelocityPID = new PIDController(0, 0, 0);
    PIDController hangPositionPID = new PIDController(0, 0, 0);


    //FIXME: Tune the Hang Up Rotational Value
    static double hangUpRot = 10;

    /**
     * @param HangRight The CAN ID of the right hang motor
     * @param HangLeft The CAN ID of the left hang motor
     */
    HangSubsystem(int HangRight, int HangLeft){

        

        //add the hang motors
        

        //set the motor gear ratio for calculations
        HardThenSoft.mHangRightEncoder.setVelocityConversionFactor(1/12); //Gear Ratio
        HardThenSoft.mHangLeftEncoder.setVelocityConversionFactor(1/12); //Gear Ratio
        HardThenSoft.mHangRightEncoder.setPositionConversionFactor(1/12); //Gear Ratio
        HardThenSoft.mHangLeftEncoder.setPositionConversionFactor(1/12); //Gear Ratio

    }

    /**
     * @param velocity The target velocity for the hang motors, unit is RPM at the spool
     */
    public void setHangVelocity(double velocity){

        //current velocity of the motors
        double currentVRight = HardThenSoft.mHangRightEncoder.getVelocity();
        double currentVLeft = HardThenSoft.mHangLeftEncoder.getVelocity();

        //get the PID outputs based on current and target velocity
        double outputRight = hangVelocityPID.calculate(currentVRight, velocity);
        double outputLeft = hangVelocityPID.calculate(currentVLeft, velocity);

        // Ensure the outputs are within the valid range of -1 to 1
        outputRight = Math.max(-1.0, Math.min(1.0, outputRight));
        outputLeft = Math.max(-1.0, Math.min(1.0, outputLeft));


        //set the motor speeds
        HardThenSoft.mHangRight.set(outputRight);
        HardThenSoft.mHangLeft.set(outputLeft);


    }

    /**
     * 
     * @param speed The target speed for the hang motors, Ranges from -1 to 1
     */
    public void setHangSpeed(double speed){
        HardThenSoft.mHangRight.set(speed);
        HardThenSoft.mHangLeft.set(speed);
    }


    //Assumes the hang encoders are zeroed at the bottom
    public void up(){
        //get the current position of the hang
        double currentPosRight = HardThenSoft.mHangRightEncoder.getPosition();
        double currentPosLeft = HardThenSoft.mHangLeftEncoder.getPosition();

        //calculate the PID outputs based on current and target position
        double outputRight = hangPositionPID.calculate(currentPosRight, hangUpRot);
        double outputLeft = hangPositionPID.calculate(currentPosLeft, hangUpRot);

        // Ensure the outputs are within the valid range of -1 to 1
        outputRight = Math.max(-1.0, Math.min(1.0, outputRight));
        outputLeft = Math.max(-1.0, Math.min(1.0, outputLeft));

        //set the motor speeds
        HardThenSoft.mHangRight.set(outputRight);
        HardThenSoft.mHangLeft.set(outputLeft);

    }   


    //Assumes the hang encoders are zeroed at the bottom
    public void down(){
        //get the current position of the hang
        double currentPosRight = HardThenSoft.mHangRightEncoder.getPosition();
        double currentPosLeft = HardThenSoft.mHangLeftEncoder.getPosition();

        // calculate the PID outputs based on current and target position
        double outputRight = hangPositionPID.calculate(currentPosRight, 0);
        double outputLeft = hangPositionPID.calculate(currentPosLeft, 0);

        // Ensure the outputs are within the valid range of -1 to 1
        outputRight = Math.max(-1.0, Math.min(1.0, outputRight));
        outputLeft = Math.max(-1.0, Math.min(1.0, outputLeft));

        //set the motor speeds
        HardThenSoft.mHangRight.set(outputRight);
        HardThenSoft.mHangLeft.set(outputLeft);

    }

    public void stop(){
        HardThenSoft.mHangRight.set(0);
        HardThenSoft.mHangLeft.set(0);
    }

    public void resetEncoder(){
        HardThenSoft.mHangRightEncoder.setPosition(0);
        HardThenSoft.mHangLeftEncoder.setPosition(0);
    }
    
}

