// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public XboxController controller = new XboxController(0);
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  HardwareSoftware robot  = new HardwareSoftware();

  
  
  
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    robot.init();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    // SmartDashboard.putNumber("P", .01);
    // SmartDashboard.putNumber("I", 0);
    // SmartDashboard.putNumber("D", 0);
    // SmartDashboard.putNumber("I Zone", 0);
    // SmartDashboard.putNumber("F", 0);
    // SmartDashboard.putNumber("Max", 0);
    // SmartDashboard.putNumber("Min", 0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    double[] funPIDNumber = new double[7];
  
    funPIDNumber[0] = .01;
    robot.delivery.constantChange(funPIDNumber[0], funPIDNumber[1], funPIDNumber[2], funPIDNumber[3], funPIDNumber[4], funPIDNumber[5], funPIDNumber[6]);
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // funPIDNumber[0] = SmartDashboard.getNumber("P", .01);
    // funPIDNumber[1] = SmartDashboard.getNumber("I", 0);
    // funPIDNumber[2] = SmartDashboard.getNumber("D", 0);
    // funPIDNumber[3] = SmartDashboard.getNumber("I Zone", 0);
    // funPIDNumber[4] = SmartDashboard.getNumber("F", 0);
    // funPIDNumber[5] = SmartDashboard.getNumber("Max", 0);
    // funPIDNumber[6] = SmartDashboard.getNumber("Min", 0);
    
    // if (controller.getAButton()) {
    //   robot.intake.setIntakeSpeed(0.5);
    // }else if (controller.getBButton()) {
    //   robot.intake.setIntakeSpeed(-0.5);
    // } else 
    if (controller.getRightBumper()) {
      // robot.delivery.deliver(100);
      robot.delivery.mDeliveryLeft.set(1);
      robot.delivery.mDeliveryRight.set(-1);
    } else if (controller.getLeftBumper()) {
      // robot.delivery.deliver(-100);
      robot.delivery.mDeliveryLeft.set(-1);
      robot.delivery.mDeliveryRight.set(1);
    } 
    else {
      robot.intake.stop();
      robot.delivery.stop();
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
