#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>

#include "DriveManipulation.h"


DriveManipulation::DriveManipulation(frc::XboxController* getController) {
    controller = getController;
}

void DriveManipulation::setNewCenterState() {
    // grab the x and y from the robot
    double x = controller->GetLeftX();
    double y = -controller->GetLeftY();
    double rotation = controller->GetRightX();

    // This holds the vector for the position or translation of the robot and each module
    double translationVector[2] = {x,y};

    // Since the rotation from the controller is from -1, to 1, we need to convert it from 0 to 2pi
    double radianMeasure = (std::numbers::pi) + (rotation * std::numbers::pi);

    // now we want to calculate the x and y components of of the rotation vector
    double swerveAngleComponentsFromRotation[4][2] = {
        {std::cos(constantRotationAngle[0]), std::sin(constantRotationAngle[0])},
        {std::cos(constantRotationAngle[1]), std::sin(constantRotationAngle[1])},
        {std::cos(constantRotationAngle[2]), std::sin(constantRotationAngle[2])},
        {std::cos(constantRotationAngle[3]), std::sin(constantRotationAngle[3])}
    };
    
    // Math dictates that we need to multiply the rotation vector by the length over the speed of the final swerve module
    
    double lengthAndWidth = 13.5;

    for (int i = 0; i < 4; i++)
    {
        swerveAngleComponentsFromRotation[i][0] *= lengthAndWidth*std::sqrt(2);
        swerveAngleComponentsFromRotation[i][1] *= lengthAndWidth*std::sqrt(2);
    }
    
    // We can now calculate the final vector for the swerve module
    double swerveModuleFinalVector[4][2];
    for (int i = 0; i < 4; i++) {
        swerveModuleFinalVector[i][0] = translationVector[0] + swerveAngleComponentsFromRotation[i][0];
        swerveModuleFinalVector[i][1] = translationVector[1] + swerveAngleComponentsFromRotation[i][1];
    }

    // Now we can calculate the angle for the swerve module
    
    for (int i = 0; i < 4; i++) {
        if (swerveModuleFinalVector[i][1] < 0) {
            swerveModuleAngles[i] = std::acos(swerveModuleFinalVector[i][0]);
        } else if (swerveModuleFinalVector[i][1] > 0) {
            swerveModuleAngles[i] = (2*std::numbers::pi - std::acos(swerveModuleFinalVector[i][0]));
        }
    }

    // Now just set the offset for the different angles of the swerve modules
    for (int i = 0; i < 4; i++) {
        swerveModuleAngles[i] += swerveAngleOffset[i] + (std::numbers::pi / 2);
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

double DriveManipulation::getSwerveModuleAngle(int module) {
    int index = 0;
    switch (module) {
        case 0:
            return frontLeft.getCurrentAngle();
            break;
        case 1:
            return frontRight.getCurrentAngle();
            break;
        case 2:
            return backLeft.getCurrentAngle();
            break;
        case 3:
            return backRight.getCurrentAngle();
            break;
        default:
            return 0;
            break;
    }
}

// double DriveManipulation::averageAngles(double angle1, double angle2, double weight1, double weight2) {
//     double angle = 0;
//     double viableAngle1[3] = {
//         angle1,
//         angle1 + std::numbers::pi * 2,
//         angle1 - std::numbers::pi * 2
//     };
//     double viableAngle2[3] = {
//         angle2,
//         angle2 + std::numbers::pi * 2,
//         angle2 - std::numbers::pi * 2
//     };
//     double differences[9] = {
//         viableAngle1[0] - viableAngle2[0],
//         viableAngle1[0] - viableAngle2[1],
//         viableAngle1[0] - viableAngle2[2],
//         viableAngle1[1] - viableAngle2[0],
//         viableAngle1[1] - viableAngle2[1],
//         viableAngle1[1] - viableAngle2[2],
//         viableAngle1[2] - viableAngle2[0],
//         viableAngle1[2] - viableAngle2[1],
//         viableAngle1[2] - viableAngle2[2]
//     };
//     double smallestDifference[2];
//     // ablsolute the entire differences array
//     for (int i = 0; i < 9; i++) {
//         differences[i] = abs(differences[i]);
//     }
//     // find the index that holds the smallest difference
//     int mIndex = std::min_element(differences, differences + 9) - differences;

//     // use the index to use the smallest difference
//     switch (mIndex)
//     {
//     case 0:
//         smallestDifference[0] = viableAngle1[0];
//         smallestDifference[1] = viableAngle2[0];
//         break;
//     case 1:
//         smallestDifference[0] = viableAngle1[0];
//         smallestDifference[1] = viableAngle2[1];
//         break;
//     case 2:
//         smallestDifference[0] = viableAngle1[0];
//         smallestDifference[1] = viableAngle2[2];
//         break;
//     case 3:
//         smallestDifference[0] = viableAngle1[1];
//         smallestDifference[1] = viableAngle2[0];
//         break;
//     case 4:
//         smallestDifference[0] = viableAngle1[1];
//         smallestDifference[1] = viableAngle2[1];
//         break;
//     case 5:
//         smallestDifference[0] = viableAngle1[1];
//         smallestDifference[1] = viableAngle2[2];
//         break;
//     case 6:
//         smallestDifference[0] = viableAngle1[2];
//         smallestDifference[1] = viableAngle2[0];
//         break;
//     case 7:
//         smallestDifference[0] = viableAngle1[2];
//         smallestDifference[1] = viableAngle2[1];
//         break;
//     case 8:
//         smallestDifference[0] = viableAngle1[2];
//         smallestDifference[1] = viableAngle2[2];
//         break;
//     default:
//         break;
//     }

//     // now we can average the angles
//     angle = (smallestDifference[0] * weight1 + smallestDifference[1] * weight2) / (weight1 + weight2);

    
//     return angle;
// }
