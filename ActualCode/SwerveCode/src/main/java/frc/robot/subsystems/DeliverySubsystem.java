package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HardThenSoft;


public class DeliverySubsystem {


    
    //FIXME: Tune the Delivery PID Values
    SparkPIDController deliveryLeftPID = HardThenSoft.mDeliveryLeft.getPIDController();
    SparkPIDController deliveryRightPID = HardThenSoft.mDeliveryRight.getPIDController();
    
    //PID Gains
    public double kP, kI, kD, kIZone, kFF, maxOutput, minOutput;
    /**
     * @param DeliveryRight The CAN ID of the right delivery motor
     * @param DeliveryLeft The CAN ID of the left delivery motor
     */
    public DeliverySubsystem(){
        //set the PID values for the delivery motors
        kP = 0.0001;
        kI = 0;
        kD = 0.0001;
        kIZone = 0;
        kFF = 0;
        maxOutput = 1;
        minOutput = -1;

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
        HardThenSoft.mDeliveryRightEncoder.setVelocityConversionFactor(1/12); //Gear Ratio
        HardThenSoft.mDeliveryLeftEncoder.setVelocityConversionFactor(1/12); //Gear Ratio


    }


/**
 * @param velocity The target velocity for the delivery motors, unit is RPM at the Flywheel
 */
    public void deliver(double velocity){

        //current velocity of the motors
        double currentVRight = HardThenSoft.mDeliveryRightEncoder.getVelocity();
        double currentVLeft = HardThenSoft.mDeliveryLeftEncoder.getVelocity();

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
        HardThenSoft.mDeliveryRight.set(0);
        HardThenSoft.mDeliveryLeft.set(0);
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
}

