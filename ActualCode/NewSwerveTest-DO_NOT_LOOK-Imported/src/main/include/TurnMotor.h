#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkFlex.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <AHRS.h>
#include <frc/controller/PIDController.h>

class TurnMotor
{
private:
    double previousState;
    double setState;
    double errorInState;
    rev::CANSparkFlex basicTurnMotor;
    ctre::phoenix6::hardware::CANcoder basicTurnEncoder;
    double difference;
    double pidStuff[3] = {.3, 0, .1};
public:
    TurnMotor(int getCanId, int getEncoderId);
    void setDesiredAngle(double radianMeasure);
    void runToState();
    double getCurrentAngle();
    double getCurrentDifference();
};
