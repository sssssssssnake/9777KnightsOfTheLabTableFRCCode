package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;

public class HangSubsystem {
    HardwareSoftware robot;

    //Right Delivery Motor
    CANSparkMax mHangRight;

    //Left Delivery Motor
    CANSparkMax mHangLeft;

    //Right Delivery Encoder
    RelativeEncoder encoderRight;

    //Left Delivery Encoder
    RelativeEncoder encoderLeft;


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
        mHangRight = new CANSparkMax(HangRight, CANSparkLowLevel.MotorType.kBrushless);
        mHangLeft = new CANSparkMax(HangLeft, CANSparkLowLevel.MotorType.kBrushless);

        //get encoders from hang motors
        encoderRight = mHangRight.getEncoder();
        encoderLeft = mHangLeft.getEncoder();

        //set the motor gear ratio for calculations
        encoderRight.setVelocityConversionFactor(1/12); //Gear Ratio
        encoderLeft.setVelocityConversionFactor(1/12); //Gear Ratio
        encoderRight.setPositionConversionFactor(1/12); //Gear Ratio
        encoderLeft.setPositionConversionFactor(1/12); //Gear Ratio

    }

    /**
     * @param velocity The target velocity for the hang motors, unit is RPM at the spool
     */
    public void setHangVelocity(double velocity){

        //current velocity of the motors
        double currentVRight = encoderRight.getVelocity();
        double currentVLeft = encoderLeft.getVelocity();

        //get the PID outputs based on current and target velocity
        double outputRight = hangVelocityPID.calculate(currentVRight, velocity);
        double outputLeft = hangVelocityPID.calculate(currentVLeft, velocity);

        // Ensure the outputs are within the valid range of -1 to 1
        outputRight = Math.max(-1.0, Math.min(1.0, outputRight));
        outputLeft = Math.max(-1.0, Math.min(1.0, outputLeft));


        //set the motor speeds
        mHangRight.set(outputRight);
        mHangLeft.set(outputLeft);


    }

    /**
     * 
     * @param speed The target speed for the hang motors, Ranges from -1 to 1
     */
    public void setHangSpeed(double speed){
        mHangRight.set(speed);
        mHangLeft.set(speed);
    }


    //Assumes the hang encoders are zeroed at the bottom
    public void up(){
        //get the current position of the hang
        double currentPosRight = encoderRight.getPosition();
        double currentPosLeft = encoderLeft.getPosition();

        //calculate the PID outputs based on current and target position
        double outputRight = hangPositionPID.calculate(currentPosRight, hangUpRot);
        double outputLeft = hangPositionPID.calculate(currentPosLeft, hangUpRot);

        // Ensure the outputs are within the valid range of -1 to 1
        outputRight = Math.max(-1.0, Math.min(1.0, outputRight));
        outputLeft = Math.max(-1.0, Math.min(1.0, outputLeft));

        //set the motor speeds
        mHangRight.set(outputRight);
        mHangLeft.set(outputLeft);

    }   


    //Assumes the hang encoders are zeroed at the bottom
    public void down(){
        //get the current position of the hang
        double currentPosRight = encoderRight.getPosition();
        double currentPosLeft = encoderLeft.getPosition();

        // calculate the PID outputs based on current and target position
        double outputRight = hangPositionPID.calculate(currentPosRight, 0);
        double outputLeft = hangPositionPID.calculate(currentPosLeft, 0);

        // Ensure the outputs are within the valid range of -1 to 1
        outputRight = Math.max(-1.0, Math.min(1.0, outputRight));
        outputLeft = Math.max(-1.0, Math.min(1.0, outputLeft));

        //set the motor speeds
        mHangRight.set(outputRight);
        mHangLeft.set(outputLeft);

    }

    public void stop(){
        mHangRight.set(0);
        mHangLeft.set(0);
    }

    public void resetEncoder(){
        encoderRight.setPosition(0);
        encoderLeft.setPosition(0);
    }
    
}

