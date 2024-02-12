#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkFlex.h>
#include <AHRS.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <numbers>

#include "TurnMotor.h"



TurnMotor::TurnMotor(int getCanId, int getEncoderId)
    : basicTurnMotor{getCanId, rev::CANSparkFlex::MotorType::kBrushless}
    , basicTurnEncoder{getEncoderId, "rio"}
{
}


void TurnMotor::setDesiredAngle(double radianMeasure)
{
    errorInState = previousState - basicTurnEncoder.GetAbsolutePosition().GetValueAsDouble() * 2 * std::numbers::pi;
    //absolute the error
    errorInState = std::abs(errorInState);

    previousState = basicTurnEncoder.GetAbsolutePosition().GetValueAsDouble() * 2 * std::numbers::pi;
    setState = radianMeasure;
    pidStuff = {.1,0,0};
}

void TurnMotor::runToState() {
    //logic is to set the new position as a "0" and then run the motor to that position
    //right now only p
    double power = 0;
    
    double whereItIs = basicTurnEncoder.GetAbsolutePosition().GetValueAsDouble() * 2 * std::numbers::pi;

    double otherValidStates[2] = {
        setState - (std::numbers::pi *2),
        setState + (std::numbers::pi *2)
    };
    
    // the encoder (whereItIs) resets to 0 every 2pi, but we can tell the power to go
    // beyond 2pi, and it wont matter until it resets to 0
    // so, we just need to set it for the closest congruent angle

    double differences[3] = {
        whereItIs - setState,
        whereItIs - otherValidStates[0],
        whereItIs - otherValidStates[1]
    };

    int smallestDifference = 0;

    // find the smallest difference
    if (std::labs(differences[0]) < std::labs(differences[1])) {
        smallestDifference = 0;
    } else if (std::labs(differences[1]) < std::labs(differences[2])) {
        smallestDifference = 1;
    } else {
        smallestDifference = 2;
    }

    // note that the negative power goes forward
    power = differences[smallestDifference] * pidStuff[0];
    
    basicTurnMotor.Set(power);
}

double TurnMotor::getCurrentAngle() {
    return basicTurnEncoder.GetAbsolutePosition().GetValueAsDouble() * 2 * std::numbers::pi;
}

double TurnMotor::getCurrentDifference() {
    return setState - (basicTurnEncoder.GetAbsolutePosition().GetValueAsDouble() * 2 * std::numbers::pi) - std::numbers::pi;
}