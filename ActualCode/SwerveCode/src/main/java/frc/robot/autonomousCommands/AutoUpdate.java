package frc.robot.autonomousCommands;


import javax.print.attribute.HashPrintRequestAttributeSet;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HardThenSoft;
import frc.robot.Robot;
import frc.robot.autonomousCommands.RunToState;


public class AutoUpdate implements Runnable{
    // public double originalX;
    // public double originalY;
    // public double originalRotation;

    double xGate = 5;
    double goSpeed = .2;
    double conversionRate = 4.17;
    double radius = 42.4264069;


    public double currentEncoderValue;


    public double newX;
    public double newY;
    public double newRotation;

    public double newEncoderValue;

    public double wheelStrafeAngles;
    double[] wheelRotationAngles = new double[4];
    private double[] swerveAngleOffset = {0, 0, Math.PI / 2 , 3 * Math.PI / 4};
    private double[] constantRotationAngle = {
      - Math.PI / 2,
      - Math.PI,
      0,
      - Math.PI * 3 / 2,
  };


    boolean isgoodToGo = true;

    public AutoUpdate(double x, double y, double rotation) {
        newX = x;
        newY = -y;
        newRotation = rotation;

        // originalX = HardThenSoft.frontLeftDriveEncoder.getPosition();
        // originalY = HardThenSoft.frontRightDriveEncoder.getPosition();
        // originalRotation = HardThenSoft.frontLeftDriveEncoder.getPosition();

        // originalX = encodersToCentimeters(originalX);
        // originalY = encodersToCentimeters(originalY);
        // originalRotation = encodersToCentimeters(originalRotation);
    }

    @Override
    /* *
     * This method is only called once per run, here is the outline of what it does:
     * 1. Calculate the strafe angles of the wheels
     * 2. Make a beeline for the new position
     * 3. Find the angle for rotation
     * 4. Rotate the robot to the new rotation
     */
    public void run() {
        HardThenSoft.autoThreadRunning = true;

        HardThenSoft.frontLeftDrive.setIdleMode(IdleMode.kBrake);
        HardThenSoft.frontRightDrive.setIdleMode(IdleMode.kBrake);
        HardThenSoft.backLeftDrive.setIdleMode(IdleMode.kBrake);
        HardThenSoft.backRightDrive.setIdleMode(IdleMode.kBrake);
        // zero the new positions relatice to the old
        // double[] newPositions = {newX - originalX, newY - originalY};
        double[] newPositions = {newX, newY};
        double angleFromPosition;
        // calculate the angle of the new position using cool cos and sin stuff
        if (newPositions[1] < Math.PI) {
            angleFromPosition = Math.acos(newPositions[0] / Math.sqrt(newPositions[0] * newPositions[0] + newPositions[1] * newPositions[1]));
        } else {
            angleFromPosition = Math.PI * 2 - Math.acos(newPositions[0] / Math.sqrt(newPositions[0] * newPositions[0] + newPositions[1] * newPositions[1]));
        }

        // set all the motor angles to the angle from the new position
        for (int i = 0; i < 4; i++) {
            wheelRotationAngles[i] = angleFromPosition + swerveAngleOffset[i] + (Math.PI / 2);
        }

        HardThenSoft.frontLeft.setDesiredAngle(wheelRotationAngles[0]);
        HardThenSoft.frontRight.setDesiredAngle(wheelRotationAngles[1]);
        HardThenSoft.backLeft.setDesiredAngle(wheelRotationAngles[2]);
        HardThenSoft.backRight.setDesiredAngle(wheelRotationAngles[3]);

        // run to state asynchonously
        HardThenSoft.killAllAsync = false;
        Thread runToState = new Thread(new RunToState(angleFromPosition));
        runToState.start();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        currentEncoderValue = HardThenSoft.frontLeftDriveEncoder.getPosition();
        newEncoderValue = currentEncoderValue + centimetersToEncoders(Math.sqrt(newPositions[0] * newPositions[0] + newPositions[1] * newPositions[1]));
        while (newEncoderValue - currentEncoderValue > .5 && isgoodToGo && HardThenSoft.autoThreadRunning) {
            if (Math.abs(currentEncoderValue - newEncoderValue) > xGate) {
                HardThenSoft.frontLeftDrive.set( goSpeed);
                HardThenSoft.frontRightDrive.set(goSpeed);
                HardThenSoft.backLeftDrive.set(  goSpeed);
                HardThenSoft.backRightDrive.set( goSpeed);
            } else {
                HardThenSoft.frontLeftDrive.set( Math.abs(newEncoderValue - currentEncoderValue) / xGate * goSpeed);
                HardThenSoft.frontRightDrive.set(Math.abs(newEncoderValue - currentEncoderValue) / xGate * goSpeed);
                HardThenSoft.backLeftDrive.set(  Math.abs(newEncoderValue - currentEncoderValue) / xGate * goSpeed);
                HardThenSoft.backRightDrive.set( Math.abs(newEncoderValue - currentEncoderValue) / xGate * goSpeed);
            }
            
            currentEncoderValue = HardThenSoft.frontLeftDriveEncoder.getPosition();

            SmartDashboard.putNumber("Current Encoder Value", currentEncoderValue);
            SmartDashboard.putNumber("New Encoder Value", newEncoderValue);
            SmartDashboard.putNumber("Angle From Position", angleFromPosition * 180 / Math.PI);

            // try {
            //     Thread.sleep(100);
            // } catch (InterruptedException e) {
            //     e.printStackTrace();
            // }
        }



        HardThenSoft.killAllAsync = true;
        HardThenSoft.frontLeftDrive.set(0);
        HardThenSoft.frontRightDrive.set(0);
        HardThenSoft.backLeftDrive.set(0);
        HardThenSoft.backRightDrive.set(0);

        HardThenSoft.frontLeft.basicTurnMotor.set(0);
        HardThenSoft.frontRight.basicTurnMotor.set(0);
        HardThenSoft.backLeft.basicTurnMotor.set(0);
        HardThenSoft.backRight.basicTurnMotor.set(0);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


        
        
        HardThenSoft.frontLeftDrive.setIdleMode(IdleMode.kCoast);
        HardThenSoft.frontRightDrive.setIdleMode(IdleMode.kCoast);
        HardThenSoft.backLeftDrive.setIdleMode(IdleMode.kCoast);
        HardThenSoft.backRightDrive.setIdleMode(IdleMode.kCoast);
        
        // now we can rotate the robot
        if (newRotation != 0) {
            // we are given the angle we want to rotate to, so we needc to calculate the distance the motor needs to go when rotoating
            double rotationDistance = newRotation * radius;

            // now we just set the rotation of the motors with a new thread
            HardThenSoft.killAllAsync = false;
            Thread rotate = new Thread(new RunToState(constantRotationAngle));
            rotate.start();

            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            HardThenSoft.frontLeftDrive.setIdleMode(IdleMode.kBrake);
            HardThenSoft.frontRightDrive.setIdleMode(IdleMode.kBrake);
            HardThenSoft.backLeftDrive.setIdleMode(IdleMode.kBrake);
            HardThenSoft.backRightDrive.setIdleMode(IdleMode.kBrake);


            currentEncoderValue = HardThenSoft.frontLeftDriveEncoder.getPosition();
            newEncoderValue = currentEncoderValue + centimetersToEncoders(rotationDistance);

            while (newEncoderValue - currentEncoderValue > .5 && isgoodToGo && HardThenSoft.autoThreadRunning) {
                if (Math.abs(currentEncoderValue - newEncoderValue) > xGate) {
                    HardThenSoft.frontLeftDrive.set( goSpeed);
                    HardThenSoft.frontRightDrive.set(goSpeed);
                    HardThenSoft.backLeftDrive.set(  goSpeed);
                    HardThenSoft.backRightDrive.set( goSpeed);
                } else {
                    HardThenSoft.frontLeftDrive.set( Math.abs(newEncoderValue - currentEncoderValue) / xGate * goSpeed);
                    HardThenSoft.frontRightDrive.set(Math.abs(newEncoderValue - currentEncoderValue) / xGate * goSpeed);
                    HardThenSoft.backLeftDrive.set(  Math.abs(newEncoderValue - currentEncoderValue) / xGate * goSpeed);
                    HardThenSoft.backRightDrive.set( Math.abs(newEncoderValue - currentEncoderValue) / xGate * goSpeed);
                }
                
                currentEncoderValue = HardThenSoft.frontLeftDriveEncoder.getPosition();

                SmartDashboard.putNumber("Current Encoder Value", currentEncoderValue);
                SmartDashboard.putNumber("New Encoder Value", newEncoderValue);
                SmartDashboard.putNumber("Angle From Position", angleFromPosition * 180 / Math.PI);
            }

            HardThenSoft.killAllAsync = true;

            
            HardThenSoft.frontLeftDrive.set(0);
            HardThenSoft.frontRightDrive.set(0);
            HardThenSoft.backLeftDrive.set(0);
            HardThenSoft.backRightDrive.set(0);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            HardThenSoft.frontLeftDrive.setIdleMode(IdleMode.kCoast);
            HardThenSoft.frontRightDrive.setIdleMode(IdleMode.kCoast);
            HardThenSoft.backLeftDrive.setIdleMode(IdleMode.kCoast);
            HardThenSoft.backRightDrive.setIdleMode(IdleMode.kCoast);
        }

        



        HardThenSoft.autoThreadRunning = false;
        Robot.runAsync = true;

    }
    
    public void setNewPosition(double x, double y, double rotation) {
        newX = x;
        newY = y;
        newRotation = rotation;
    }
    
    // returns the velocity in centimeters per second
    public double velocityConverter(double encoderData) {
        double gearRatio = 6.75;
        double wheelDiameter = 10.16;
        return encoderData * ( 1 / gearRatio) * wheelDiameter;
    }

    public double encodersToCentimeters(double encoderData) {
        // double gearRatio = 6.75;
        // double wheelDiameter = 10.16;
        // double encoderTicks = 1;
        // return encoderData * ( 1 / gearRatio) * wheelDiameter * (1 / encoderTicks);
        return encoderData * conversionRate;               
    }

    public double centimetersToEncoders(double centimeters) {
        // double gearRatio = 6.75;
        // double wheelDiameter = 10.16;
        // double encoderTicks = 1;
        // return centimeters * gearRatio * (1 / wheelDiameter) * encoderTicks;
        return centimeters * (1 / conversionRate);
    }

    
}
