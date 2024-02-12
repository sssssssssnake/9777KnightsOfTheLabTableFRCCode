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

    // swerveModuleAngles[0] = controller->GetRightX() * std::numbers::pi;
    // swerveModuleAngles[1] = controller->GetRightX() * std::numbers::pi;
    // swerveModuleAngles[2] = controller->GetRightX() * std::numbers::pi;
    // swerveModuleAngles[3] = controller->GetRightX() * std::numbers::pi;

    // because the controller is from -1 to 1, and the code made it from -pi to pi,
    // the rotationRadians is from -pi to pi, so we need to convert it to 0 to 2pi
    for (int i = 0; i < 4; i++) {
        swerveModuleAngles[i] += std::numbers::pi;
    }
        
    // get the drive x and y values
    for (int i = 0; i < 4; i++) {
        swerveDriveDesiredXandY[i][0] = x;
        swerveDriveDesiredXandY[i][1] = y;
    }

    // to avoid divide by zero errors we will grab the hypotenuse of the x and y values
    // and then divide the x and y values by the hypotenuse
    double hypotenuse = std::sqrt(x * x + y * y);
    if (hypotenuse != 0) {
        for (int i = 0; i < 4; i++) {
            swerveDriveDesiredXandY[i][0] /= hypotenuse;
            swerveDriveDesiredXandY[i][1] /= hypotenuse;
        }
    }

    // we will now determine the angles from only the x values, no y
    // we cannot use tan because it is undefined at pi/2 and 3pi/2, so we can only use one of the values

    // we will use the arccosine to determine the angle
    // we will use the x value as the adjacent side, and the hypotenuse as the hypotenuse
    double anglesDerivedFromPosition[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) {
        anglesDerivedFromPosition[i] = std::acos(swerveDriveDesiredXandY[i][0]);
    }

    // we will now look at the angles from the default roation (swerveModuleDefaultAngles)

    swerveModuleAngles[0] = swerveModuleDefaultAngles[0];
    swerveModuleAngles[1] = swerveModuleDefaultAngles[1];
    swerveModuleAngles[2] = swerveModuleDefaultAngles[2];
    swerveModuleAngles[3] = swerveModuleDefaultAngles[3];
    
    // rotationRadians is the vector for the default rotation

    double proportionOfPositionToRotation = 0;

    proportionOfPositionToRotation = hypotenuse / proportionOfPositionToRotation;

    // now we have the two angles, we need to average them using the proportion just calculated
    // (p*Pos + (1/p)*Rot) / 2
    for (int i = 0; i < 4; i++) {
        swerveModuleAngles[i] = (proportionOfPositionToRotation * anglesDerivedFromPosition[i] + (1 / proportionOfPositionToRotation) * rotationRadians) / 2;
    }

    // now do the same avereage for the speeds
    for (int i = 0; i < 4; i++) {
        swerveModuleSpeeds[i] = (proportionOfPositionToRotation * hypotenuse + (1 / proportionOfPositionToRotation) * rotationRadians) / 2;
    }


    

    
    
}

void DriveManipulation::runToState() {
    frontLeft.setDesiredAngle(swerveModuleAngles[0]);
    frontRight.setDesiredAngle(swerveModuleAngles[1]);
    backLeft.setDesiredAngle(swerveModuleAngles[2]);
    backRight.setDesiredAngle(swerveModuleAngles[3]);

    frontLeft.runToState();
    frontRight.runToState();
    backLeft.runToState();
    backRight.runToState();
}