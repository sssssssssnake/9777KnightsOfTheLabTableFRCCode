// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomousCommands.AutoUpdate;
import frc.robot.autonomousCommands.PositionThread;
import frc.robot.autonomousCommands.RunOuttakeAuto;
import frc.robot.teleopSwerve.DriveManipulation;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax; 
import cameraLogic.CameraLogic;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public static XboxController controller = new XboxController(0);
  private DriveManipulation drive = new DriveManipulation(controller);
  double frontRightDriveEncoderNumber = 0;
  boolean precisionMode = false;

  boolean autoAlign = false;
  double[] specialAlignmentNumbers = new double[3];


  List<Thread> autonomoustCommands = new ArrayList<Thread>();
  public static boolean runAsync = true;
  int counterforAsync = 0;

  public Robot() {
    super(.05);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    frontRightDriveEncoderNumber = HardThenSoft.frontRightDriveEncoder.getPosition();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("frontLeft", HardThenSoft.frontLeft.basicTurnEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("frontRight", HardThenSoft.frontRight.basicTurnEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("backLeft", HardThenSoft.backLeft.basicTurnEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("backRight", HardThenSoft.backRight.basicTurnEncoder.getAbsolutePosition().getValueAsDouble());

    SmartDashboard.putNumber("frontLeftDrive Stuff", HardThenSoft.frontRightDriveEncoder.getPosition() - frontRightDriveEncoderNumber);
    SmartDashboard.putNumber("miracleMagicX", specialAlignmentNumbers[0]);
    SmartDashboard.putNumber("miracleMagicY", specialAlignmentNumbers[1]);
    SmartDashboard.putNumber("miracleMagicZ", specialAlignmentNumbers[2]);
    SmartDashboard.putNumber("GyroRadians", HardThenSoft.navx.getAngle() * (Math.PI / 180) + Math.PI);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    switch (m_autoSelected) {
      case kCustomAuto:
        
        break;
      case kDefaultAuto:
        autonomoustCommands.add(new Thread(new PositionThread(0, 100, 1)));
        // autonomoustCommands.add(new Thread(new AutoUpdate(-150, 0, Math.PI)));
        // autonomoustCommands.add(new Thread(new RunOuttakeAuto()));
        break; 
      default:
        break;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
      // Put default auto code here
        if (runAsync) {
          if(counterforAsync >= autonomoustCommands.size()){
            break;
          } else {
            autonomoustCommands.get(counterforAsync++).start();
            runAsync = false;
          }
        }
        break;
    }

    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    HardThenSoft.killAllAsync = true;
    HardThenSoft.autoThreadRunning = false;

    HardThenSoft.mHangLeft.setIdleMode( CANSparkMax.IdleMode.kBrake);
    HardThenSoft.mHangRight.setIdleMode(CANSparkMax.IdleMode.kBrake);

    HardThenSoft.frontLeftDrive.setIdleMode( CANSparkMax.IdleMode.kBrake);
    HardThenSoft.frontRightDrive.setIdleMode(CANSparkMax.IdleMode.kBrake);
    HardThenSoft.backLeftDrive.setIdleMode(  CANSparkMax.IdleMode.kBrake);
    HardThenSoft.backRightDrive.setIdleMode( CANSparkMax.IdleMode.kBrake);


  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {


    if(controller.getYButtonReleased()){
      autoAlign = true;
    }

    if(!autoAlign && !HardThenSoft.autoThreadRunning){

      //Control the swerve drive
      drive.setNewCenterState();
      drive.runToState(precisionMode);
      //Enable/Disable Precision Drive Mode
      if (controller.getAButtonReleased()) { 
        precisionMode = !precisionMode;
      }

      //Control the Delivery System
      if (controller.getPOV() == 0) {
        HardThenSoft.mDeliveryLeft.set(-1);
        HardThenSoft.mDeliveryRight.set(1);
        HardThenSoft.mIntake.set(.5);
      } 
      else if(controller.getPOV() == 90) {
        HardThenSoft.mDeliveryLeft.set(-0.2475);
        HardThenSoft.mDeliveryRight.set(0.17);
        HardThenSoft.mIntake.set(.5);
      }
      else if(controller.getPOV() == 180) {
        HardThenSoft.mDeliveryLeft.set(1);
        HardThenSoft.mDeliveryRight.set(-1);
      }
      else if(controller.getPOV() == -1) {
        HardThenSoft.mDelivery.stop();
        HardThenSoft.mIntake.set(0);
      }



      //Control the Intake
      if (controller.getRightTriggerAxis() > .1) {
        HardThenSoft.mIntake.set(controller.getRightTriggerAxis());
      } 
      else if (controller.getLeftTriggerAxis() > .1) {
        HardThenSoft.mIntake.set(-controller.getLeftTriggerAxis());
      } 
      else {
        HardThenSoft.mIntake.set(0);
      }



      //Control the Hang System
      if (controller.getLeftBumper()) {
        HardThenSoft.mHangLeft.set(-.5);
        HardThenSoft.mHangRight.set(.5);
      } 
      else if (controller.getRightBumper()) {
        HardThenSoft.mHangLeft.set(.5);
        HardThenSoft.mHangRight.set(-.5);
      } 
      else {
        HardThenSoft.mHangLeft.set(0);
        HardThenSoft.mHangRight.set(0);
      }

      if (controller.getBackButton()) {
        HardThenSoft.gyroOffset = - HardThenSoft.navx.getAngle() / 180 * Math.PI;
      }

      
    } //Automatic Alignment Control
    else if(!HardThenSoft.autoThreadRunning){
      CameraLogic.autoAlign();
      autoAlign = false;
    }
    
    if (controller.getStartButton()) {
      specialAlignmentNumbers = cameraLogic.CameraLogic.postXYZ();
      HardThenSoft.killAllAsync = true;
      HardThenSoft.autoThreadRunning = false;
    }
    
    
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
