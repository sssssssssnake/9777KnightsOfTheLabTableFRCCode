package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DeliverySubsystem {
    HardwareSoftware robot;

    //Right Delivery Motor
    CANSparkMax mDeliveryRight;

    //Left Delivery Motor
    CANSparkMax mDeliveryLeft;

    //Right Delivery Encoder
    RelativeEncoder encoderRight;

    //Left Delivery Encoder
    RelativeEncoder encoderLeft;


    
    //FIXME: Tune the Delivery PID Values
    SparkPIDController deliveryLeftPID;
    SparkPIDController deliveryRightPID;
    
    //PID Gains
    public double kP, kI, kD, kIZone, kFF, maxOutput, minOutput;
    /**
     * @param DeliveryRight The CAN ID of the right delivery motor
     * @param DeliveryLeft The CAN ID of the left delivery motor
     */
    DeliverySubsystem(int DeliveryRight, int DeliveryLeft){
        
        
        
        //add the delivery motors
        mDeliveryRight = new CANSparkMax(DeliveryRight, CANSparkLowLevel.MotorType.kBrushless);
        mDeliveryLeft = new CANSparkMax(DeliveryLeft, CANSparkLowLevel.MotorType.kBrushless);
        
        
        //get encoders from delivery motors
        encoderRight = mDeliveryRight.getEncoder();
        encoderLeft = mDeliveryLeft.getEncoder();
        
        deliveryLeftPID = mDeliveryLeft.getPIDController();
        deliveryRightPID = mDeliveryRight.getPIDController();


        deliveryLeftPID.setP(kP);
        deliveryLeftPID.setI(kI);
        deliveryLeftPID.setD(kD);
        deliveryLeftPID.setIZone(kIZone);
        deliveryLeftPID.setFF(kFF);
        deliveryLeftPID.setOutputRange(minOutput, maxOutput);
        
        // now set the PID values for the right motor
        deliveryRightPID.setP(kP);
        deliveryRightPID.setI(kI);
        deliveryRightPID.setD(kD);
        deliveryRightPID.setIZone(kIZone);
        deliveryRightPID.setFF(kFF);
        deliveryRightPID.setOutputRange(minOutput, maxOutput);



        //set the motor gear ratios for calculations
        encoderRight.setVelocityConversionFactor(1/12); //Gear Ratio
        encoderLeft.setVelocityConversionFactor(1/12); //Gear Ratio


    }


/**
 * @param velocity The target velocity for the delivery motors, unit is RPM at the Flywheel
 */
    public void deliver(double velocity){

        //current velocity of the motors
        double currentVRight = encoderRight.getVelocity();
        double currentVLeft = encoderLeft.getVelocity();

        //get the PID outputs based on current and target velocity
        deliveryLeftPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
        deliveryRightPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);

        SmartDashboard.putNumber("Left Delivery Motor Speed", currentVLeft);
        SmartDashboard.putNumber("Right Delivery Motor Speed", currentVRight);

        // Ensure the outputs are within the valid range of -1 to 1
        // outputRight = Math.max(-1.0, Math.min(1.0, outputRight));
        // outputLeft = Math.max(-1.0, Math.min(1.0, outputLeft));


    }
    public void stop(){
        mDeliveryRight.set(0);
        mDeliveryLeft.set(0);
    }


    public void constantChange(double p, double i, double d, double iZone, double f, double max, double min) {
        deliveryLeftPID.setP(p);
        deliveryLeftPID.setI(i);
        deliveryLeftPID.setD(d);
        deliveryLeftPID.setIZone(iZone);
        deliveryLeftPID.setFF(f);
        deliveryLeftPID.setOutputRange(min, max);
        
        // now set the PID values for the right motor
        deliveryRightPID.setP(p);
        deliveryRightPID.setI(i);
        deliveryRightPID.setD(d);
        deliveryRightPID.setIZone(iZone);
        deliveryRightPID.setFF(f);
        deliveryRightPID.setOutputRange(min, max);

    }

    public void setSpeed(double speed) {
        mDeliveryRight.set(speed);
        mDeliveryLeft.set(speed);
    }
}

