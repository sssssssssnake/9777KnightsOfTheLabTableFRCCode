package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;

public class DeliverySubsystem {
    HardwareMap robot = new HardwareMap();

    //Right Delivery Motor
    CANSparkMax mDeliveryRight;

    //Left Delivery Motor
    CANSparkMax mDeliveryLeft;

    //Right Delivery Encoder
    RelativeEncoder encoderRight;

    //Left Delivery Encoder
    RelativeEncoder encoderLeft;


    //FIXME: Tune the Delivery PID Values
    PIDController deliveryPID = new PIDController(0, 0, 0);

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
        double outputRight = deliveryPID.calculate(currentVRight, velocity);
        double outputLeft = deliveryPID.calculate(currentVLeft, velocity);

        // Ensure the outputs are within the valid range of -1 to 1
        outputRight = Math.max(-1.0, Math.min(1.0, outputRight));
        outputLeft = Math.max(-1.0, Math.min(1.0, outputLeft));

        // Set the motor speeds
        mDeliveryRight.set(outputRight);
        mDeliveryLeft.set(outputLeft);


    }
    public void stop(){
        mDeliveryRight.set(0);
        mDeliveryLeft.set(0);
    }
}

