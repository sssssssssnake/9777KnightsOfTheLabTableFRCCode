package frc.robot.autonomousCommands;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.HardThenSoft;
import frc.robot.Robot;

public class PositionThread implements Runnable{

    // cosntants regarding movement
    double stopError = .3;
    double xGate = 10;
    double goSpeed = .3;
    double conversionRate = 5;
    double radius = 42.4264069;
    double rotationGoSpeed = .2;
    double stopRadianError = .15;
    double rotationGate = 1.5;

    public double currentEncoderValue;

    // variables for the new position
    public double newX;
    public double newY;
    public double newRotation;
    public boolean runIntake;

    public double newEncoderValue;



    // constants for the swerve drive
    public double wheelStrafeAngles;
    double[] wheelRotationAngles = new double[4];
    private double[] constantRotationAngle = {
      - Math.PI / 2,
      - Math.PI,
      0,
      - Math.PI * 3 / 2,
  };

  public PositionThread(double x, double y, double rotation) {
    this(x, y, rotation, false);
  }

  public PositionThread(double x, double y, double rotation, boolean runIntake) {
    this.newX = x;
    this.newY = -y;
    this.newRotation = rotation;
    this.runIntake = runIntake;
  }

    @Override
    public void run() {
        // set the flags for the thread running in HardThenSoft
        HardThenSoft.autoThreadRunning = true;

        // make sure the robot is in the right mode
        HardThenSoft.frontLeftDrive.setIdleMode(IdleMode.kBrake);
        HardThenSoft.frontRightDrive.setIdleMode(IdleMode.kBrake);
        HardThenSoft.backLeftDrive.setIdleMode(IdleMode.kBrake);
        HardThenSoft.backRightDrive.setIdleMode(IdleMode.kBrake);

        // get the current encoder value and store it
        currentEncoderValue = getNewEncoderValue();

        // calculate the angle for the position translation and the distance to the position
        double[] newPositions = {newX, newY};
        double angleFromPosition;
        // calculate the angle of the new position using cool cos and sin stuff
        if (newPositions[1] < Math.PI) {
            angleFromPosition = Math.acos(newPositions[0] / Math.sqrt(newPositions[0] * newPositions[0] + newPositions[1] * newPositions[1]));
        } else {
            angleFromPosition = Math.PI * 2 - Math.acos(newPositions[0] / Math.sqrt(newPositions[0] * newPositions[0] + newPositions[1] * newPositions[1]));
        }

        // calculate the distance to the new position
        double distanceToPosition = Math.sqrt(newPositions[0] * newPositions[0] + newPositions[1] * newPositions[1]);

        // new we want to drive there

        // set the wheel angles to the angle of the position
        startSetWheelAngles(angleFromPosition);
        sleepyNightNight(300);



        // drive to the position
        newEncoderValue = currentEncoderValue + centimetersToEncoders(distanceToPosition);

        while (Math.abs(newEncoderValue - getNewEncoderValue()) > stopError && !HardThenSoft.killAllAsync) {
            if (Math.abs(newEncoderValue - getNewEncoderValue()) < xGate) {
                HardThenSoft.frontLeftDrive.set( goSpeed);
                HardThenSoft.frontRightDrive.set(goSpeed);
                HardThenSoft.backLeftDrive.set(  goSpeed);
                HardThenSoft.backRightDrive.set( goSpeed);
            } else {
                HardThenSoft.frontLeftDrive.set( goSpeed * (Math.abs(newEncoderValue - currentEncoderValue) / xGate));
                HardThenSoft.frontRightDrive.set(goSpeed * (Math.abs(newEncoderValue - currentEncoderValue) / xGate));
                HardThenSoft.backLeftDrive.set(  goSpeed * (Math.abs(newEncoderValue - currentEncoderValue) / xGate));
                HardThenSoft.backRightDrive.set( goSpeed * (Math.abs(newEncoderValue - currentEncoderValue) / xGate));
            }
            currentEncoderValue = getNewEncoderValue();
        }

        // stop the robot
        HardThenSoft.killAllAsync = true;
        HardThenSoft.frontLeftDrive.set(0);
        HardThenSoft.frontRightDrive.set(0);
        HardThenSoft.backLeftDrive.set(0);
        HardThenSoft.backRightDrive.set(0);

        sleepyNightNight(1000);

        // now we can rotate the robot using the gyro
        if (newRotation == 0) {
            return;
        }
        
        // set the wheel angles to the angle of the rotation
        startSetWheelAngles(constantRotationAngle);
        sleepyNightNight(300);

        double oldGyroValue = (HardThenSoft.navx.getAngle() * Math.PI / 180) + Math.PI;
        double newGyroValue = addAngle(oldGyroValue, newRotation);
        double angleDifference = findBestAngleDifference(oldGyroValue, newGyroValue);

        while (Math.abs(angleDifference) > stopRadianError && !HardThenSoft.killAllAsync) {
            if (Math.abs(angleDifference) > rotationGate) {
                HardThenSoft.frontLeftDrive.set( rotationGoSpeed * -findSign(angleDifference));
                HardThenSoft.frontRightDrive.set(rotationGoSpeed * -findSign(angleDifference));
                HardThenSoft.backLeftDrive.set(  rotationGoSpeed * -findSign(angleDifference));
                HardThenSoft.backRightDrive.set( rotationGoSpeed * -findSign(angleDifference));
            } else {
                HardThenSoft.frontLeftDrive.set( rotationGoSpeed * (angleDifference / rotationGate) * -findSign(angleDifference));
                HardThenSoft.frontRightDrive.set(rotationGoSpeed * (angleDifference / rotationGate) * -findSign(angleDifference));
                HardThenSoft.backLeftDrive.set(  rotationGoSpeed * (angleDifference / rotationGate) * -findSign(angleDifference));
                HardThenSoft.backRightDrive.set( rotationGoSpeed * (angleDifference / rotationGate) * -findSign(angleDifference));
            }

            oldGyroValue = (HardThenSoft.navx.getAngle() * Math.PI / 180) + Math.PI;
            angleDifference = findBestAngleDifference(oldGyroValue, newGyroValue);
            
        }

        // stop the robot
        HardThenSoft.killAllAsync = true;
        HardThenSoft.frontLeftDrive.set(0);
        HardThenSoft.frontRightDrive.set(0);
        HardThenSoft.backLeftDrive.set(0);
        HardThenSoft.backRightDrive.set(0);


        HardThenSoft.autoThreadRunning = false;
        Robot.runAsync = true;
    }

    
    // this gets the average of all the encoders from the drive motors
    private double getNewEncoderValue() {
        double encoderValue = 0;
        encoderValue += HardThenSoft.frontLeftDriveEncoder.getPosition();
        encoderValue += HardThenSoft.frontRightDriveEncoder.getPosition();
        encoderValue += HardThenSoft.backLeftDriveEncoder.getPosition();
        encoderValue += HardThenSoft.backRightDriveEncoder.getPosition();

        return encoderValue / 4;
    }

    private void startSetWheelAngles(double[] wheelAngles) {
        HardThenSoft.killAllAsync = false;
        Thread runToState = new Thread(new RunToState(wheelAngles));
        runToState.start();
    }

    // overloading the function to take in a single angle
    private void startSetWheelAngles(double wheelAngle) {
        HardThenSoft.killAllAsync = false;
        Thread runToState = new Thread(new RunToState(wheelAngle));
        runToState.start();
    }


    private double centimetersToEncoders(double centimeters) {
        return centimeters * (1 / conversionRate);
    }

    private void sleepyNightNight(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private double addAngle(double firstAngle, double secondAngle) {
        double newAngle = firstAngle + secondAngle;
        if (newAngle == 0) {
            return firstAngle;
        }
        while (newAngle < 0) {
            newAngle += Math.PI * 2;
        }
        while (newAngle > Math.PI * 2) {
            newAngle -= Math.PI * 2;
        }
        return newAngle;
    }

    private double findBestAngleDifference(double firstAngle, double secondAngle) {

        double[] differences = {
            firstAngle - secondAngle,
            firstAngle - (secondAngle + Math.PI * 2),
            firstAngle - (secondAngle - Math.PI * 2)
        };

        int smallestDifference = 0;

        if (Math.abs(differences[0]) < Math.abs(differences[1]) && Math.abs(differences[0]) < Math.abs(differences[2])) {
            smallestDifference = 0;
        } else if (Math.abs(differences[1]) < Math.abs(differences[2]) && Math.abs(differences[1]) < Math.abs(differences[0])) {
            smallestDifference = 1;
        } else if (Math.abs(differences[2]) < Math.abs(differences[0]) && Math.abs(differences[2]) < Math.abs(differences[1])) {
            smallestDifference = 2;
        }

        return differences[smallestDifference];
    }

    int findSign(double value) {
        if (value == 0) {
            return 0;
        } else if (value <0) {
            return -1;
        } else if (value > 0) {
            return 1;
        } else {
            return 1;
        }
    }
    
}
