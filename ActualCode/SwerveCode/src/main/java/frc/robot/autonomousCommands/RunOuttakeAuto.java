package frc.robot.autonomousCommands;

import frc.robot.HardThenSoft;

public class RunOuttakeAuto implements Runnable {
    boolean runIntake = false;

    @Override
    public void run() {
        HardThenSoft.mDeliveryLeft.set(-1);
        HardThenSoft.mDeliveryRight.set(1);
        Thread intake = new Thread(new RunIntakeWithSwerve(1, 2));

        while (HardThenSoft.mDeliveryLeftEncoder.getVelocity() > -4000 && !HardThenSoft.killAllAsync) {
            
        }

        if (runIntake) {
            intake.start();
        }

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        HardThenSoft.mDeliveryLeft.set(0);
        HardThenSoft.mDeliveryRight.set(0);

    }

    public RunOuttakeAuto(boolean withIntake) {
        runIntake = withIntake;
    }

    
}
