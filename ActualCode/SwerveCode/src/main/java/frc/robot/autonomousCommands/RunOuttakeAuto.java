package frc.robot.autonomousCommands;

import frc.robot.HardThenSoft;

public class RunOuttakeAuto implements Runnable {

    @Override
    public void run() {
        HardThenSoft.mDeliveryLeft.set(-1);
        HardThenSoft.mDeliveryRight.set(1);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        HardThenSoft.mDeliveryLeft.set(0);
        HardThenSoft.mDeliveryRight.set(0);

    }
    
}
