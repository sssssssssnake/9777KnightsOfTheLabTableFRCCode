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
    double differenceInState = setState - (basicTurnEncoder.GetAbsolutePosition().GetValueAsDouble() * 2 * std::numbers::pi);

    //convert the range of differenceInState to be between 1 and -1
    differenceInState = ((differenceInState -std::numbers::pi)/ std::numbers::pi);

    //multiply the difference by the p value
    power = differenceInState * pidStuff[0];

    if (setState > basicTurnEncoder.GetAbsolutePosition().GetValueAsDouble() * 2 * std::numbers::pi) {
        basicTurnMotor.Set(power);
    } else if (setState < basicTurnEncoder.GetAbsolutePosition().GetValueAsDouble() * 2 * std::numbers::pi)
    {
        basicTurnMotor.Set(-power);
    }
    

    //set the motor to the power
    basicTurnMotor.Set(power);
}

double TurnMotor::getCurrentAngle() {
    return basicTurnEncoder.GetAbsolutePosition().GetValueAsDouble() * 2 * std::numbers::pi;
}

double TurnMotor::getCurrentDifference() {
    return setState - (basicTurnEncoder.GetAbsolutePosition().GetValueAsDouble() * 2 * std::numbers::pi) - std::numbers::pi;
}