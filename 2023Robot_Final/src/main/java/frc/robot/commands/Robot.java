// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

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

  boolean LowLevelEnabled=false;
  boolean MidLevelEnabled=false;
  boolean UpperLevelEnabled=false;

  double phi_degrees;
  double theta_degrees;

  Joystick stick;

  RoboticArm arm;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    stick=new Joystick(0);
    arm=new RoboticArm();
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

    arm.X=-19.5;
    arm.Y=6.5;

    arm.computeArmAngles(arm.X,arm.Y);

    phi_degrees=arm.Phi*180.0/Math.PI;
    theta_degrees=arm.Theta*180.0/Math.PI;

    System.out.printf("\nPhi = %.3f  Theta = %.3f degrees",phi_degrees,theta_degrees);

    arm.upperRotation_complete=false;
    arm.lowerRotation_complete=false;

    

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


    
    if(arm.lowerRotation_complete==false)    {
      arm.rotateLowerArm(theta_degrees);  
    }
  
    if(arm.upperRotation_complete==false)  {
      arm.rotateUpperArm(phi_degrees+theta_degrees); 
          
    }
    
    




  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    arm.MoveComplete=true;
    LowLevelEnabled=false;
    MidLevelEnabled=false;
    UpperLevelEnabled=false;
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

      //  We detect a button press and enable the
      //  appropriate movement.  We want to make sure that any
      //  movements in progress are complete.
     
      
      if((stick.getRawButtonPressed(8)==true)&&(arm.MoveComplete==true)) {
      
        MidLevelEnabled=true;
        LowLevelEnabled=false;
        UpperLevelEnabled=false;
        arm.moveInit=1; 
        arm.MoveComplete=false;
        System.out.printf("\nButton 8 pressed.\n");

      }
      if(MidLevelEnabled==true)  {
          arm.processMidLevel();
      }

      if((stick.getRawButtonPressed(9)==true)&&(arm.MoveComplete==true)) {
        
        MidLevelEnabled=false;
        LowLevelEnabled=true;
        UpperLevelEnabled=false;
        arm.moveInit=1; 
        arm.MoveComplete=false;
        System.out.printf("\nButton 9 pressed.\n");

      }
      if(LowLevelEnabled==true)  {
          arm.processLowLevel();
      }

      if((stick.getRawButtonPressed(10)==true)&&(arm.MoveComplete==true)) {
        
        MidLevelEnabled=false;
        LowLevelEnabled=false;
        UpperLevelEnabled=true;
        arm.moveInit=1; 
        arm.MoveComplete=false;
        System.out.printf("\nButton 10 pressed.\n");

      }
      if(UpperLevelEnabled==true)  {
          arm.processUpperLevel();
          //arm.rotateUpperArm_CCW(20.0);
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
