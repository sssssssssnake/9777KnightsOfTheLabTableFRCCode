#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>
#include "TurnMotor.h"

class DriveManipulation
{
private:

    double swerveModulePositions[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}}; // in meters
    double swerveModuleAngles[4] = {0, 0, 0, 0}; // in radians
    double swerveModuleSpeeds[4] = {0, 0, 0, 0}; // in meters per second
    
    double swerveDriveDesiredXandY[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}}; // in meters

    //swerve default angles
    double swerveModuleDefaultAngles[4] = {std::numbers::pi * 3 / 4, std::numbers::pi * 1 / 4, std::numbers::pi * 5 / 4, std::numbers::pi * 7 / 4};


    frc::XboxController* controller;
    TurnMotor frontLeft{8, 16};
    TurnMotor frontRight{10, 17};
    TurnMotor backLeft{12, 18};
    TurnMotor backRight{14, 19};
public:
    DriveManipulation(frc::XboxController* getController);
    void setNewCenterState();
    void runToState();
    
};

