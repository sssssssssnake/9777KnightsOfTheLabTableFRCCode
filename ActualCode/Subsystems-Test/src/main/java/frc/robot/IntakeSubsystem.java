package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;

public class IntakeSubsystem {
    HardwareSoftware robot ;

    //Intake Motor
    CANSparkMax mIntake;


    //Intake Encoder
    RelativeEncoder encoder;



    //FIXME: Tune the Intake PID Values
    PIDController intakePID = new PIDController(0, 0, 0);

    /**
     * @param intakeCAN The CAN ID of the intake motor
     */
    IntakeSubsystem(int intakeCAN){

        

        //add the delivery motors
        mIntake = new CANSparkMax(intakeCAN, CANSparkLowLevel.MotorType.kBrushless);

        //get encoders from intake motor
        encoder = mIntake.getEncoder();

        //set the motor gear ratio for calculations
        encoder.setVelocityConversionFactor(1/12); //Gear Ratio
    }
    

    /**
     * @param velocity The target velocity for the intake motor, unit is RPM at the intake Axle
     */
    //TODO: Possibly don't need velocity control, just use controller trigger
    public void setIntakeVelocity(double velocity){

        //current velocity of the motor
        double currentV = encoder.getVelocity();

        //get the PID outputs based on current and target velocity
        double output = intakePID.calculate(currentV, velocity);

        // Ensure the outputs are within the valid range of -1 to 1
        if (output > 1.0) {
            output = 1.0;
        } else if (output < -1.0) {
            output = -1.0;
        }

        // Set the motor speed
        mIntake.set(output);

    }

    /**
     * 
     * @param speed The target speed for the intake motor, Ranges from -1 to 1
     */
    public void setIntakeSpeed(double speed){
        mIntake.set(speed);
    }

    public void stop(){
        mIntake.set(0);
    }
}

