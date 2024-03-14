package frc.robot.autonomousCommands;

import frc.robot.HardThenSoft;

public class RunIntakeWithSwerve implements Runnable{
    double rotation;
    double time;



    public RunIntakeWithSwerve(double rotation, double time) {
        this.rotation = rotation;
        this.time = time;
    }

    @Override
    public void run() {
        HardThenSoft.intakeRunning = true;
        
        HardThenSoft.mIntake.set(.5);
        try {
            Thread.sleep((long) (time * 1000));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        HardThenSoft.mIntake.set(0);

    }
    
}
