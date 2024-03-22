package frc.robot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DeliverySubsystem;
import frc.robot.teleopSwerve.TurnMotor;
import com.kauailabs.navx.frc.AHRS;

public class HardThenSoft {
    public static AHRS navx = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);
    public static XboxController controller;
    public static TurnMotor frontLeft  = new TurnMotor( 9, 16);
    public static TurnMotor frontRight = new TurnMotor(11, 17);
    public static TurnMotor backLeft   = new TurnMotor(13, 18);
    public static TurnMotor backRight  = new TurnMotor(15, 19);
    public static CANSparkFlex frontLeftDrive  = new CANSparkFlex( 8, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    public static CANSparkFlex frontRightDrive = new CANSparkFlex(10, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    public static CANSparkFlex backLeftDrive   = new CANSparkFlex(12, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    public static CANSparkFlex backRightDrive  = new CANSparkFlex(14, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    public static CANSparkMax mIntake = new CANSparkMax(5, CANSparkLowLevel.MotorType.kBrushless);

    public static CANSparkMax mDeliveryRight = new CANSparkMax(6, CANSparkLowLevel.MotorType.kBrushless);
    public static CANSparkMax mDeliveryLeft  = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless);

    public static CANSparkMax mHangRight = new CANSparkMax(4, CANSparkLowLevel.MotorType.kBrushless);
    public static CANSparkMax mHangLeft  = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless);

    public static RelativeEncoder frontLeftDriveEncoder = frontLeftDrive.getEncoder();
    public static RelativeEncoder frontRightDriveEncoder = frontRightDrive.getEncoder();
    public static RelativeEncoder backLeftDriveEncoder = backLeftDrive.getEncoder();
    public static RelativeEncoder backRightDriveEncoder = backRightDrive.getEncoder();

    public static RelativeEncoder mIntakeEncoder = mIntake.getEncoder();

    public static RelativeEncoder mDeliveryRightEncoder = mDeliveryRight.getEncoder();
    public static RelativeEncoder mDeliveryLeftEncoder  = mDeliveryLeft.getEncoder();

    public static RelativeEncoder mHangRightEncoder = mHangRight.getEncoder();
    public static RelativeEncoder mHangLeftEncoder  = mHangLeft.getEncoder();


    public static DeliverySubsystem mDelivery = new DeliverySubsystem();

    public static boolean killAllAsync = false;
    public static boolean autoThreadRunning = false;
    public static boolean intakeRunning = false;

    public static double gyroOffset = 0.0;

    public static double frontLeftAngleOffset = 107;
    public static double frontRightAngleOffset = -53;
    public static double backLeftAngleOffset = -65;
    public static double backRightAngleOffset = -55;

    public HardThenSoft() {
        mHangLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mHangRight.setIdleMode(CANSparkMax.IdleMode.kBrake);

        frontLeftDrive.setSmartCurrentLimit(55);
        frontRightDrive.setSmartCurrentLimit(55);
        backLeftDrive.setSmartCurrentLimit(55);
        backRightDrive.setSmartCurrentLimit(55);

    }

}
