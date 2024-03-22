package frc.robot.autonomousCommands;


import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HardThenSoft;
import frc.robot.Robot;

public class PositionThread implements Runnable{

    // cosntants regarding movement
    double stopError = .3;
    double xGate = 7;
    double goSpeed = .7;
    double conversionRate = 5;
    double radius = 42.4264069;
    double rotationGoSpeed = .4;
    double stopRadianError = .08;
    double rotationGate = 2;

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
    this.newRotation = rotation * (1.06);
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
        HardThenSoft.killAllAsync = false;
        startSetWheelAngles(angleFromPosition);
        sleepyNightNight(500);
        
        if (runIntake) {
            HardThenSoft.intakeRunning = true;
            Thread runIntake = new Thread(new RunIntakeWithSwerve(0, 1.4));
            runIntake.start();
            goSpeed = .2;
        }
        
        
        // drive to the position
        currentEncoderValue = getNewEncoderValue();
        newEncoderValue = currentEncoderValue + centimetersToEncoders(distanceToPosition);

        while (Math.abs(newEncoderValue - getNewEncoderValue()) > stopError && HardThenSoft.autoThreadRunning) {
            if (Math.abs(newEncoderValue - getNewEncoderValue()) > xGate) {
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

        sleepyNightNight(400);

        // now we can rotate the robot using the gyro
        if (newRotation == 0) {
            HardThenSoft.autoThreadRunning = false;
            Robot.runAsync = true;
            return;
        }
        
        // set the wheel angles to the angle of the rotation
        HardThenSoft.killAllAsync = false;
        startSetWheelAngles(constantRotationAngle);
        sleepyNightNight(500);

        double currentGyroValue = HardThenSoft.navx.getAngle() * (Math.PI / 180)  + Math.PI;
        currentGyroValue = zeroToTwoPI(currentGyroValue);
        double setGyroAngle = currentGyroValue + newRotation;
        setGyroAngle = zeroToTwoPI(setGyroAngle);

        // the positive power rotates the robot positively angle wise, but the angles are reversed.
        while (Math.abs(setGyroAngle - currentGyroValue) > stopRadianError && HardThenSoft.autoThreadRunning){
            if (Math.abs(setGyroAngle- currentGyroValue) > rotationGate) {
                HardThenSoft.frontLeftDrive.set( rotationGoSpeed * -findSign(findBestAngleDifference(currentGyroValue, setGyroAngle)));
                HardThenSoft.frontRightDrive.set(rotationGoSpeed * -findSign(findBestAngleDifference(currentGyroValue, setGyroAngle)));
                HardThenSoft.backLeftDrive.set(  rotationGoSpeed * -findSign(findBestAngleDifference(currentGyroValue, setGyroAngle)));
                HardThenSoft.backRightDrive.set( rotationGoSpeed * -findSign(findBestAngleDifference(currentGyroValue, setGyroAngle)));
            } else {
                HardThenSoft.frontLeftDrive.set( rotationGoSpeed * -findBestAngleDifference(currentGyroValue, setGyroAngle) / rotationGate);
                HardThenSoft.frontRightDrive.set(rotationGoSpeed * -findBestAngleDifference(currentGyroValue, setGyroAngle) / rotationGate);
                HardThenSoft.backLeftDrive.set(  rotationGoSpeed * -findBestAngleDifference(currentGyroValue, setGyroAngle) / rotationGate);
                HardThenSoft.backRightDrive.set( rotationGoSpeed * -findBestAngleDifference(currentGyroValue, setGyroAngle) / rotationGate);
            }

            currentGyroValue = HardThenSoft.navx.getAngle() * (Math.PI / 180)  + Math.PI;
            currentGyroValue = zeroToTwoPI(currentGyroValue);

            SmartDashboard.putNumber("currentGyroValue", currentGyroValue);
            SmartDashboard.putNumber("setGyroAngle", setGyroAngle);
            SmartDashboard.putNumber("difference", findBestAngleDifference(currentGyroValue, setGyroAngle));
            
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

    private double zeroToTwoPI(double angle) {
        if (angle == 0) {
            return 0;
        }

        while (angle > (Math.PI * 2)) {
            angle -= Math.PI * 2;
        }
        while (angle < 0) {
            angle += Math.PI * 2;
        }
        return angle;
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
