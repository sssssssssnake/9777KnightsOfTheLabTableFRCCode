package frc.robot.autonomousCommands;

import frc.robot.HardThenSoft;

public class RunToState implements Runnable {
    private double[] wheelRotationAngles = new double[4];
    public void run() {
        while (true && !HardThenSoft.killAllAsync) {
            HardThenSoft.frontLeft.setDesiredAngle(wheelRotationAngles[0] + (13 * Math.PI / 180));
            HardThenSoft.frontRight.setDesiredAngle(wheelRotationAngles[1] + Math.PI - (25 * Math.PI / 180));
            HardThenSoft.backLeft.setDesiredAngle(wheelRotationAngles[2] + (10 * Math.PI / 180));
            HardThenSoft.backRight.setDesiredAngle(wheelRotationAngles[3] - (12 * Math.PI / 180));

            HardThenSoft.frontLeft.runToState();
            HardThenSoft.frontRight.runToState();
            HardThenSoft.backLeft.runToState();
            HardThenSoft.backRight.runToState();
        }
    }

    public RunToState(double angle) {
        for (int i = 0; i < 4; i++) {
            wheelRotationAngles[i] = angle + Math.PI / 4;
        }
    }

    public RunToState(double[] angles) {
        for (int i = 0; i < 4; i++) {
            wheelRotationAngles[i] = angles[i];
        }
    }
    
}