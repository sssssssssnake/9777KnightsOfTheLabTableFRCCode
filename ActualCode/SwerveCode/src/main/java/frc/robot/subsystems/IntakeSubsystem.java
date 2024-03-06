package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.HardThenSoft;

public class IntakeSubsystem {
    

    //FIXME: Tune the Intake PID Values
    PIDController intakePID = new PIDController(0, 0, 0);

    /**
     * @param intakeCAN The CAN ID of the intake motor
     */
    IntakeSubsystem(int intakeCAN){

        //set the motor gear ratio for calculations
        HardThenSoft.mIntakeEncoder.setVelocityConversionFactor(1/12); //Gear Ratio
    }
    

    /**
     * @param velocity The target velocity for the intake motor, unit is RPM at the intake Axle
     */
    //TODO: Possibly don't need velocity control, just use controller trigger
    public void setIntakeVelocity(double velocity){

        //current velocity of the motor
        double currentV = HardThenSoft.mIntakeEncoder.getVelocity();

        //get the PID outputs based on current and target velocity
        double output = intakePID.calculate(currentV, velocity);

        // Ensure the outputs are within the valid range of -1 to 1
        if (output > 1.0) {
            output = 1.0;
        } else if (output < -1.0) {
            output = -1.0;
        }

        // Set the motor speed
        HardThenSoft.mIntake.set(output);

    }

    /**
     * 
     * @param speed The target speed for the intake motor, Ranges from -1 to 1
     */
    public void setIntakeSpeed(double speed){
        HardThenSoft.mIntake.set(speed);
    }

    public void stop(){
        HardThenSoft.mIntake.set(0);
    }
}

