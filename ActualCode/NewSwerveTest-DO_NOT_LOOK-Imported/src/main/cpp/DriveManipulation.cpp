#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>

#include "DriveManipulation.h"


DriveManipulation::DriveManipulation(frc::XboxController* getController) {
    controller = getController;
}

void DriveManipulation::setNewCenterState() {
    x = controller->GetLeftX();
    y = -controller->GetLeftY();
    rotation = controller->GetRightX();

    // V_p
    double positionVector[2] = {x, y};
    // v_s
    double newRotationVector[4][2];
    double constantRotationVector[4][2];

    // we need to get the x and y components of the rotation vector
    for (int i = 0; i < 4; i++) {
        newRotationVector[i][0] = std::cos(constantRotationAngle[i]);
        newRotationVector[i][1] = std::sin(constantRotationAngle[i]);
    }

    // we then need to scale the rotation vector by the rotation value
    for (int i = 0; i < 4; i++) {
        newRotationVector[i][0] *= rotation;
        newRotationVector[i][1] *= rotation;
    }

    // now that we have the two scaled and set vector arrays, we can add them together

    // V_f
    double finalVector[4][2];
    for (int i = 0; i < 4; i++) {
        finalVector[i][0] = positionVector[0] + newRotationVector[i][0];
        finalVector[i][1] = positionVector[1] + newRotationVector[i][1];
    }

    // now we can get the angle from the position vector
    for (int i = 0; i < 4; i++) {
        // we dont use atan2 because we want to keep the angle between 0 and 2pi
        if (finalVector[i][1] > 0) {
            swerveModuleAngles[i] = std::acos(finalVector[i][0] / std::sqrt(finalVector[i][0] * finalVector[i][0] + finalVector[i][1] * finalVector[i][1]));
        } else if (finalVector[i][1] < 0) {
            swerveModuleAngles[i] = std::numbers::pi * 2 - std::acos(finalVector[i][0] / std::sqrt(finalVector[i][0] * finalVector[i][0] + finalVector[i][1] * finalVector[i][1]));
        } 
    }

    // dont forget to add the offset
    for (int i = 0; i < 4; i++) {
        swerveModuleAngles[i] += swerveAngleOffset[i] + (std::numbers::pi /2);
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
