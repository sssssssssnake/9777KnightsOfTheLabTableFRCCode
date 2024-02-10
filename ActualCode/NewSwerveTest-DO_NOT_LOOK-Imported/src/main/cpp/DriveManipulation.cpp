#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>

#include "DriveManipulation.h"


DriveManipulation::DriveManipulation(frc::XboxController* getController) {
    controller = getController;
}

void DriveManipulation::setNewCenterState() {
    //get the x and y values from the controller
    double x = controller->GetLeftX() * .1;
    double y = controller->GetLeftY() * .1;
    double rotationZeroToOne = controller->GetRightX();

    // convert the rotationZeroToOne to radians
    double rotationRadians = rotationZeroToOne * std::numbers::pi;

    swerveModuleAngles[0] = controller->GetRightX();
    swerveModuleAngles[1] = controller->GetRightX();
    swerveModuleAngles[2] = controller->GetRightX();
    swerveModuleAngles[3] = controller->GetRightX();

    if (!(x == 0) || !(rotationRadians == 0)) {// calculate the desired x and y for each module
    for (int i = 0; i < 4; i++) {
        swerveDriveDesiredXandY[i][0] = x;
        swerveDriveDesiredXandY[i][1] = y;
    }

    // convert the rotation and position vectors into 1 vector for the entire module
    for (int i = 0; i < 4; i++) {
        swerveDriveDesiredSpeeds[i] = sqrt(pow(swerveDriveDesiredXandY[i][0], 2) + pow(swerveDriveDesiredXandY[i][1], 2));
    }

    // calculate the desired angle for each module
    for (int i = 0; i < 4; i++) {
        swerveDriveDesiredAngles[i] = atan2(swerveDriveDesiredXandY[i][1], swerveDriveDesiredXandY[i][0]);
        //account for the rotation of the module as well as position
        swerveDriveDesiredAngles[i] = atan2(rotationRadians, swerveDriveDesiredAngles[i]);
    }}
    
}

void DriveManipulation::runToState() {
    frontLeft.setDesiredAngle(swerveDriveDesiredAngles[0]);
    frontRight.setDesiredAngle(swerveDriveDesiredAngles[1]);
    backLeft.setDesiredAngle(swerveDriveDesiredAngles[2]);
    backRight.setDesiredAngle(swerveDriveDesiredAngles[3]);

    frontLeft.runToState();
    frontRight.runToState();
    backLeft.runToState();
    backRight.runToState();
}