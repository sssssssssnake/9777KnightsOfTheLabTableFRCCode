package frc.robot.autonomousCommands;

import frc.robot.HardThenSoft;
import frc.robot.Robot;

public class RunOuttakeAuto implements Runnable {
    boolean runIntake = false;
    boolean runCheck = true;
    boolean valueCheck = false;

    @Override
    public void run() {
        if (valueCheck) {
            HardThenSoft.autoThreadRunning = true;
        }
        HardThenSoft.mDeliveryLeft.set(-1);
        HardThenSoft.mDeliveryRight.set(1);
        Thread intake = new Thread(new RunIntakeWithSwerve(1, 2));

        while (runCheck && !HardThenSoft.killAllAsync) {
            if (HardThenSoft.mDeliveryLeft.getEncoder().getVelocity() < -4000) {
                runCheck = false;
                if (runIntake) {
                    intake.start();
                }
            }
        }


        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        HardThenSoft.mDeliveryLeft.set(0);
        HardThenSoft.mDeliveryRight.set(0);

        if (valueCheck) {
            HardThenSoft.autoThreadRunning = false;
            Robot.runAsync = true;
        }

    }

    public RunOuttakeAuto(boolean withIntake) {
        runIntake = withIntake;
    }
    public RunOuttakeAuto(boolean withIntake, boolean asSeperateThread) {
        runIntake = withIntake;
        valueCheck = asSeperateThread;
    }

    
}
