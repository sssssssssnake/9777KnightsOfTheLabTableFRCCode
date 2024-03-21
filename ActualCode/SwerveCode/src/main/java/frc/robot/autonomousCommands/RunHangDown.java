package frc.robot.autonomousCommands;


import frc.robot.HardThenSoft;
import frc.robot.Robot;

public class RunHangDown implements Runnable {
    long time;
    boolean moveOnToNextThread;

    @Override
    public void run() {
        if (moveOnToNextThread) {
            Robot.runAsync = true;
        }
        HardThenSoft.mHangLeft.set(.5);
        HardThenSoft.mHangRight.set(-.5);

        try {
            Thread.sleep(time * 1000);
        } catch (Exception e) {
            e.printStackTrace();
        }

        HardThenSoft.mHangLeft.set(0);
        HardThenSoft.mHangRight.set(0);

        if (!moveOnToNextThread) {
            Robot.runAsync = true;
        }
    }

    public RunHangDown(long seconds) {
        time = seconds;
    }

    public RunHangDown(long seconds, boolean skipToNextThread) {
        time = seconds;
        moveOnToNextThread = skipToNextThread;
    }
    
}
