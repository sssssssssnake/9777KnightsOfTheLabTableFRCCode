package frc.robot.autonomousCommands;

import frc.robot.HardThenSoft;
import frc.robot.Robot;

public class RunIntakeWithSwerve implements Runnable{
    double rotation;
    double time;
    boolean runSeperate = false;



    public RunIntakeWithSwerve(double rotation, double time) {
        this.rotation = rotation;
        this.time = time;
    }
    public RunIntakeWithSwerve(double rotation, double time, boolean runSeperate) {
        this.rotation = rotation;
        this.time = time;
        this.runSeperate = runSeperate;
    }


    @Override
    public void run() {
        HardThenSoft.intakeRunning = true;
        if (runSeperate) {
            HardThenSoft.autoThreadRunning = true;
        }
        
        HardThenSoft.mIntake.set(-.5 * rotation);
        try {
            Thread.sleep((long) (time * 1000));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        HardThenSoft.mIntake.set(0);
        if (runSeperate) {
            HardThenSoft.autoThreadRunning = true;
            Robot.runAsync = true;
        }

    }
    
}
