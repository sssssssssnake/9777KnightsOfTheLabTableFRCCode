package frc.robot.teleopSwerve;

import com.revrobotics.CANSparkFlex;
import com.ctre.phoenix6.hardware.*;
import java.lang.Math;

public class TurnMotor {
    
    private double previousState;
    private double setState;
    private double errorInState;
    public CANSparkFlex basicTurnMotor;
    public CANcoder basicTurnEncoder;
    private double[] pidStuff = {.3, 0, .1};

    public TurnMotor(int getCanId, int getEncoderId){
        basicTurnMotor = new CANSparkFlex(getCanId, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        basicTurnEncoder = new CANcoder(getEncoderId);
    
    }
    public void setDesiredAngle(double radianMeasure) {
    
    
        errorInState = previousState - basicTurnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        //absolute the error
        errorInState = Math.abs(errorInState);
    
        previousState = basicTurnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        setState = radianMeasure;
        while (setState >= Math.PI * 2)
        {
            setState -= Math.PI * 2;
        }
        while (setState < 0)
        {
            setState += Math.PI * 2;
        }
    
    }

    public void runToState() {
        //logic is to set the new position as a "0" and then run the motor to that position
        //right now only p
        double power = 0;
        
        double whereItIs = basicTurnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    
        double[] otherValidStates = {
            setState + (Math.PI *2),
            setState - (Math.PI *2)
        };
        
        // the encoder (whereItIs) resets to 0 every 2pi, but we can tell the power to go
        // beyond 2pi, and it wont matter until it resets to 0
        // so, we just need to set it for the closest congruent angle
    
        double[] differences = {
            whereItIs - setState,
            whereItIs - otherValidStates[0],
            whereItIs - otherValidStates[1]
        };
    
        int smallestDifference = 0;
    
        // find the smallest difference
        if (Math.abs(differences[0]) < Math.abs(differences[1]) && Math.abs(differences[0]) < Math.abs(differences[2])) {
            smallestDifference = 0;
        } else if (Math.abs(differences[1]) < Math.abs(differences[2]) && Math.abs(differences[1]) < Math.abs(differences[0])) {
            smallestDifference = 1;
        } else if (Math.abs(differences[2]) < Math.abs(differences[0]) && Math.abs(differences[2]) < Math.abs(differences[1])) {
            smallestDifference = 2;
        }
    
        // note that the negative power goes forward
        double powerProportion = pidStuff[0] * differences[smallestDifference];
    
        power = powerProportion;
        
        basicTurnMotor.set(power);
    }
}
