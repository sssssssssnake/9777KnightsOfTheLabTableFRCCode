package frc.robot.autonomousCommands;

import frc.robot.HardThenSoft;

public class RunIntakeWithSwerve implements Runnable{
    double x;
    double y;
    double rotation;
    double time;



    public RunIntakeWithSwerve(double x, double y, double rotation, double time) {
        this.x = x;
        this.y = y;
        this.rotation = rotation;
        this.time = time;
    }

    @Override
    public void run() {
        HardThenSoft.intakeRunning = true;
        AutoUpdate movUpdate = new AutoUpdate(x, y, rotation);
        Thread movThread = new Thread(movUpdate);
        
        movThread.start();

        HardThenSoft.mIntake.set(.5);
        try {
            Thread.sleep((long) (time * 1000));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        HardThenSoft.mIntake.set(0);

        if (HardThenSoft.autoThreadRunning) {
            HardThenSoft.autoThreadRunning = false;
        } else {
            HardThenSoft.killAllAsync = true;
        }
        
        HardThenSoft.intakeRunning = false;
    }
    
}
