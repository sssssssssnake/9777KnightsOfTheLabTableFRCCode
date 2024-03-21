// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomousCommands.PositionThread;
import frc.robot.autonomousCommands.RunHangDown;
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
  private static final String defaultBlueWorks = "Blue aligned";
  private static final String backAndForth = "Out Of The Way Score";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public static XboxController controller = new XboxController(0);
  private DriveManipulation drive = new DriveManipulation(controller);
  boolean precisionMode = false;
  boolean dpadPrecisionMode = false;

  boolean autoAlign = false;
  double[] specialAlignmentNumbers = new double[3];
  double[] degrees = new double[4];


  List<Thread> autonomoustCommands = new ArrayList<Thread>();
  public static boolean runAsync = true;
  int counterforAsync = 0;

  public Robot() {
    super(.04);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Working blue auto", defaultBlueWorks);
    m_chooser.addOption("Out Of The Way Auto", backAndForth);
    m_chooser.addOption("Default Auto", kDefaultAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
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

    SmartDashboard.putNumber("miracleMagicX", specialAlignmentNumbers[0]);
    SmartDashboard.putNumber("miracleMagicY", specialAlignmentNumbers[1]);
    SmartDashboard.putNumber("miracleMagicZ", specialAlignmentNumbers[2]);
    SmartDashboard.putNumber("GyroRadians", HardThenSoft.navx.getAngle() * (Math.PI / 180) + Math.PI);
    SmartDashboard.putNumber("funnyRPM", HardThenSoft.mDeliveryLeftEncoder.getVelocity());

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
      case backAndForth:{
        autonomoustCommands.add(new Thread(new RunHangDown(4, true)));
        autonomoustCommands.add(new Thread(new RunOuttakeAuto(true, true)));
        autonomoustCommands.add(new Thread(new PositionThread(0, -150, 0)));
        autonomoustCommands.add(new Thread(new PositionThread(0, -60, 0, true)));
        autonomoustCommands.add(new Thread(new PositionThread(0, 210, 0)));
        autonomoustCommands.add(new Thread(new RunOuttakeAuto(true, true)));
        break;
      }
      case defaultBlueWorks: {
        autonomoustCommands.add(new Thread(new RunHangDown(4, true)));
        autonomoustCommands.add(new Thread(new RunOuttakeAuto(true, true)));
        autonomoustCommands.add(new Thread(new PositionThread(0, -80, 0)));
        autonomoustCommands.add(new Thread(new PositionThread(0, -60,0, true)));
        autonomoustCommands.add(new Thread(new PositionThread(0, 130, 0)));
        autonomoustCommands.add(new Thread(new RunOuttakeAuto(true, true)));
        autonomoustCommands.add(new Thread(new PositionThread(145, -60, 0)));
        autonomoustCommands.add(new Thread(new PositionThread(0, -80,0, true)));
        autonomoustCommands.add(new Thread(new PositionThread(-145, 140, 0)));
        autonomoustCommands.add(new Thread(new RunOuttakeAuto(true, true)));
        break; 
      }
      case kDefaultAuto: {
          autonomoustCommands.add(new Thread(new RunHangDown(4, true)));
          autonomoustCommands.add(new Thread(new PositionThread(0, -150, 0)));
          break;
      }
      default:
        break;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (runAsync) {
      if(counterforAsync >= autonomoustCommands.size()){
      } else {
        autonomoustCommands.get(counterforAsync++).start();
        runAsync = false;
      }
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
    
    
    //Control the swerve drive
    drive.setNewCenterState();
    drive.runToState(precisionMode || dpadPrecisionMode);
    
    if(controller.getYButtonReleased()){
      autoAlign = true;
      CameraLogic.autoAlign();
    }

    if(!autoAlign && !HardThenSoft.autoThreadRunning){

      //Enable/Disable Precision Drive Mode
      if (controller.getAButtonReleased()) { 
        precisionMode = !precisionMode;
      }
      //Control the Intake
      if (controller.getRightTriggerAxis() > .1) {
        HardThenSoft.mIntake.set(controller.getRightTriggerAxis());
      } else if (controller.getLeftTriggerAxis() > .1) {
        HardThenSoft.mIntake.set(-controller.getLeftTriggerAxis());
      } else {
        HardThenSoft.mIntake.set(0);
      }



      //Control the Delivery System
      if (controller.getPOV() == 0) {
        if(HardThenSoft.mDeliveryLeft.getEncoder().getVelocity() < -4700) {
          HardThenSoft.mIntake.set(-.5);
        }
        HardThenSoft.mDeliveryLeft.set(-1);
        HardThenSoft.mDeliveryRight.set(1);
        dpadPrecisionMode = true;
      } else if(controller.getPOV() == 90) {
        HardThenSoft.mIntake.set(-.5);
        HardThenSoft.mDeliveryLeft.set(-0.14);
        HardThenSoft.mDeliveryRight.set(0.14);
        dpadPrecisionMode = true;
      } else if(controller.getPOV() == 180) {
        HardThenSoft.mDeliveryLeft.set(1);
        HardThenSoft.mDeliveryRight.set(-1);
        dpadPrecisionMode = true;
      } else {
        HardThenSoft.mDelivery.stop();
        dpadPrecisionMode = false;
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
        HardThenSoft.gyroOffset -= Math.PI / 4;
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
