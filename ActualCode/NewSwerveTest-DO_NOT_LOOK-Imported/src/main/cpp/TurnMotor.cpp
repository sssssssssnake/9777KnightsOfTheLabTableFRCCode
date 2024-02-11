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
    double whereItIsOffset = whereItIs - std::numbers::pi;
    // get the error between where it is now, and where it needs to be

    double difference = whereItIs - setState;
    double differenceOffset = whereItIsOffset - setState;

    // the module can go either way so we only need to worry about moving it within 180 degrees
    bool backwards = (std::abs(differenceOffset) > std::numbers::pi/2);

    // if it is backwards, we use the offset difference, otherwise we use the normal difference
    // we need them to switch if and only in the case of the offset difference
    if (backwards) {
        basicTurnMotor.Set(-differenceOffset * pidStuff[0]);
    } else {
        basicTurnMotor.Set(difference * pidStuff[0]);
    }
}

double TurnMotor::getCurrentAngle() {
    return basicTurnEncoder.GetAbsolutePosition().GetValueAsDouble() * 2 * std::numbers::pi;
}

double TurnMotor::getCurrentDifference() {
    return setState - (basicTurnEncoder.GetAbsolutePosition().GetValueAsDouble() * 2 * std::numbers::pi) - std::numbers::pi;
}