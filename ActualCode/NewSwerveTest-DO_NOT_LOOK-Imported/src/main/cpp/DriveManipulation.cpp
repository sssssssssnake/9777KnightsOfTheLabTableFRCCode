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

    // apply the deadband of .02
    if (x < .02 && x > -.02) {
        x = 0;
    }
    if (y < .02 && y > -.02) {
        y = 0;
    }


    // now I want how much the robot is rotating
    // apply the deadband of .02
    if (rotation < .02 && rotation > -.02) {
        rotation = 0;
    }

    // If everything is 0, then we don't want to do anything
    if (x == 0 && y == 0 && rotation == 0) {
        for (int i = 0; i < 4; i++) {
            swerveModuleSpeeds[i] = 0;
        }
    } else {
        // Now we can update the angles and speeds of the swerve modules

        // First we need to find the angle of the robot from x and y (we eventually need the rotation)
        // to prevent any errors, the logic is as follows:
        // angle is acosx if y is positive, 2pi - acosx if y is negative

        double positionAngle = 0;
        double hypotenuse = sqrt(x * x + y * y);
        if (y > 0) {
            positionAngle = acos(x / hypotenuse);
        } else {
            positionAngle = (2 * std::numbers::pi) - acos(x / hypotenuse);
        }

        // Now we need to find the angle of the robot from the rotation
        // we can just grab the swerveModuleDefaultAngles and average it with positionAngle
        // the hypotenuse is how strong the foreward and backward is, so we need to average it with the strength of the rotation

        swerveModuleAngles[0] = averageAngles(positionAngle, swerveModuleDefaultAngles[0], hypotenuse, rotation) + swerveAngleOffset[0] + (std::numbers::pi / 2);
        swerveModuleAngles[1] = averageAngles(positionAngle, swerveModuleDefaultAngles[1], hypotenuse, rotation) + swerveAngleOffset[1] + (std::numbers::pi / 2);
        swerveModuleAngles[2] = averageAngles(positionAngle, swerveModuleDefaultAngles[2], hypotenuse, rotation) + swerveAngleOffset[2] + (std::numbers::pi / 2);
        swerveModuleAngles[3] = averageAngles(positionAngle, swerveModuleDefaultAngles[3], hypotenuse, rotation) + swerveAngleOffset[3] + (std::numbers::pi / 2);

        // Now we need to find the speed of the swerve modules
        


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

double DriveManipulation::averageAngles(double angle1, double angle2, double weight1, double weight2) {
    double angle = 0;
    double viableAngle1[3] = {
        angle1,
        angle1 + std::numbers::pi * 2,
        angle1 - std::numbers::pi * 2
    };
    double viableAngle2[3] = {
        angle2,
        angle2 + std::numbers::pi * 2,
        angle2 - std::numbers::pi * 2
    };
    double differences[9] = {
        viableAngle1[0] - viableAngle2[0],
        viableAngle1[0] - viableAngle2[1],
        viableAngle1[0] - viableAngle2[2],
        viableAngle1[1] - viableAngle2[0],
        viableAngle1[1] - viableAngle2[1],
        viableAngle1[1] - viableAngle2[2],
        viableAngle1[2] - viableAngle2[0],
        viableAngle1[2] - viableAngle2[1],
        viableAngle1[2] - viableAngle2[2]
    };
    double smallestDifference[2];
    // ablsolute the entire differences array
    for (int i = 0; i < 9; i++) {
        differences[i] = abs(differences[i]);
    }
    // find the index that holds the smallest difference
    int mIndex = std::min_element(differences, differences + 9) - differences;

    // use the index to use the smallest difference
    switch (mIndex)
    {
    case 0:
        smallestDifference[0] = viableAngle1[0];
        smallestDifference[1] = viableAngle2[0];
        break;
    case 1:
        smallestDifference[0] = viableAngle1[0];
        smallestDifference[1] = viableAngle2[1];
        break;
    case 2:
        smallestDifference[0] = viableAngle1[0];
        smallestDifference[1] = viableAngle2[2];
        break;
    case 3:
        smallestDifference[0] = viableAngle1[1];
        smallestDifference[1] = viableAngle2[0];
        break;
    case 4:
        smallestDifference[0] = viableAngle1[1];
        smallestDifference[1] = viableAngle2[1];
        break;
    case 5:
        smallestDifference[0] = viableAngle1[1];
        smallestDifference[1] = viableAngle2[2];
        break;
    case 6:
        smallestDifference[0] = viableAngle1[2];
        smallestDifference[1] = viableAngle2[0];
        break;
    case 7:
        smallestDifference[0] = viableAngle1[2];
        smallestDifference[1] = viableAngle2[1];
        break;
    case 8:
        smallestDifference[0] = viableAngle1[2];
        smallestDifference[1] = viableAngle2[2];
        break;
    default:
        break;
    }

    // now we can average the angles
    angle = (smallestDifference[0] * weight1 + smallestDifference[1] * weight2) / (weight1 + weight2);

    
    return angle;
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