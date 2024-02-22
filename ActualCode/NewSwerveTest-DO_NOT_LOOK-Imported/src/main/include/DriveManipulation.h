#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>
#include <rev/CANSparkFlex.h>
#include "TurnMotor.h"

class DriveManipulation
{
private:

    double swerveModuleAngles[4] = {0, 0, 0, 0}; // in radians
    double swerveModuleSpeeds[4] = {0, 0, 0, 0}; // in meters per second
    // Angle for the translation of the robot/ module derived from position
    double angleFromPosition;
    
    double x;
    double y;
    double rotation;


    // These are constants that needn't be changed by any code

    // offset for where motors are in radians
    double swerveAngleOffset[4] = {4.2107776536,5.595962,6.062292,0.503146};

    /**
     * If the robot rotates, there are different angles that the robot uses to rotate.
     * In the case that the robot is to rotate, we need to use the vector that aligns itself with the robot's rotation
     * These are the default radian measure of the robot
     * @category Constant Expression */
    double constantRotationAngle[4] = {
        - std::numbers::pi * 3 /4,
        - std::numbers::pi / 4,
        - std::numbers::pi * 5 / 4 ,
        - std::numbers::pi * 7 / 4
    };

    frc::XboxController* controller;
    TurnMotor frontLeft{9, 16};
    TurnMotor frontRight{11, 17};
    TurnMotor backLeft{13, 18};
    TurnMotor backRight{15, 19};

    rev::CANSparkFlex frontLeftDrive{8, rev::CANSparkLowLevel::MotorType::kBrushless};
    rev::CANSparkFlex frontRightDrive{10, rev::CANSparkLowLevel::MotorType::kBrushless};
    rev::CANSparkFlex backLeftDrive{12, rev::CANSparkLowLevel::MotorType::kBrushless};
    rev::CANSparkFlex backRightDrive{14, rev::CANSparkLowLevel::MotorType::kBrushless};

public:
    DriveManipulation(frc::XboxController* getController);
    void setNewCenterState();
    void runToState();
    double getSwerveModuleAngle(int module);

    // double averageAngles(double angle1, double angle2, double weight1, double weight2);
};

