// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import com.ctre.phoenix.motorcontrol.ControlMode;

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

  boolean RevLowLevelEnabled=false;
  boolean RevMidLevelEnabled=false;
  boolean RevUpperLevelEnabled=false;
  boolean ResetEnabled=false;

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
     
      //arm.upperMotor.set(ControlMode.PercentOutput, stick.getRawAxis(1));
      //arm.lowerMotor.set(ControlMode.PercentOutput, stick.getRawAxis(5));

      //processArmButtons();

      if (stick.getRawButton(1)){
        arm.move2Position(30, 30);
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

  /////////////////////////////////////////////////////////////////
  //  Function:  void processArmButtons()
  /////////////////////////////////////////////////////////////////
  //
  //  Purpose:  Processes joystick buttons to manipulate the arm(s)
  //            to predetermined positions.
  //
  //  Arguments:void
  //
  //  Returns:  void
  //
  //  Remarks: The actual coordinates of each move will need to
  //           be empirically determined due to backlash in the
  //           arm geartrain(s).
  //           Actual buttons used TBD
  //
  /////////////////////////////////////////////////////////////////
  void processArmButtons()
  {

    //  Here is the reset function
    if((stick.getRawButtonPressed(5)==true) && (arm.MoveComplete==true))  {
        MidLevelEnabled=false;
        LowLevelEnabled=false;
        UpperLevelEnabled=false;
        RevMidLevelEnabled=false;
        RevLowLevelEnabled=false;
        RevUpperLevelEnabled=false;
        ResetEnabled=true;
        arm.moveInit=1; 
        arm.MoveComplete=false;
        System.out.printf("\nButton 5 pressed.\n");
        arm.reset();

      }
      if(ResetEnabled==true)  {
        arm.reset();
      }
      
      //  Low position
      if((stick.getRawButtonReleased(4)==true) &&
      (stick.getRawButtonPressed(1)==true) &&
      (arm.MoveComplete==true))  {
        MidLevelEnabled=false;
        LowLevelEnabled=true;
        UpperLevelEnabled=false;
        RevMidLevelEnabled=false;
        RevLowLevelEnabled=false;
        RevUpperLevelEnabled=false;
        ResetEnabled=false;
        arm.moveInit=1; 
        arm.MoveComplete=false;
        System.out.printf("\nButton 4 released.\n");
        System.out.printf("\nButton 1 pressed.\n");

      }
      if(LowLevelEnabled==true)  {
          arm.processLowLevel();
      }

      //  Mid position
      if((stick.getRawButtonReleased(4)==true) &&
      (stick.getRawButtonPressed(2)==true) &&
      (arm.MoveComplete==true))  {
        MidLevelEnabled=true;
        LowLevelEnabled=false;
        UpperLevelEnabled=false;
        RevMidLevelEnabled=false;
        RevLowLevelEnabled=false;
        RevUpperLevelEnabled=false;
        ResetEnabled=false;
        arm.moveInit=1; 
        arm.MoveComplete=false;
        System.out.printf("\nButton 4 released.\n");
        System.out.printf("\nButton 2 pressed.\n");

      }
      if(MidLevelEnabled==true)  {
          arm.processMidLevel();
      }      

      //  Upper position
      if((stick.getRawButtonReleased(4)==true) &&
      (stick.getRawButtonPressed(3)==true) &&
      (arm.MoveComplete==true))  {
        MidLevelEnabled=false;
        LowLevelEnabled=false;
        UpperLevelEnabled=true;
        RevMidLevelEnabled=false;
        RevLowLevelEnabled=false;
        RevUpperLevelEnabled=false;
        ResetEnabled=false;
        arm.moveInit=1; 
        arm.MoveComplete=false;
        System.out.printf("\nButton 4 released.\n");
        System.out.printf("\nButton 3 pressed.\n");

      }
      if(UpperLevelEnabled==true)  {
          arm.processUpperLevel();
      }

      //  Reverse lower position
      if((stick.getRawButtonPressed(4)==true) &&
        (stick.getRawButtonPressed(1)==true) &&
        (arm.MoveComplete==true))  {

          MidLevelEnabled=false;
          LowLevelEnabled=false;
          UpperLevelEnabled=false;
          RevMidLevelEnabled=false;
          RevLowLevelEnabled=true;
          RevUpperLevelEnabled=false;
          ResetEnabled=false;
          arm.moveInit=1; 
          arm.MoveComplete=false;
          System.out.printf("\nButton 4 pressed.\n");
          System.out.printf("\nButton 1 pressed.\n");

      }
      if(RevLowLevelEnabled==true)  {
        arm.processRevLowLevel();
      }

      //  Reverse mid position
      if((stick.getRawButtonPressed(4)==true) &&
        (stick.getRawButtonPressed(2)==true) &&
        (arm.MoveComplete==true))  {

          MidLevelEnabled=false;
          LowLevelEnabled=false;
          UpperLevelEnabled=false;
          RevMidLevelEnabled=true;
          RevLowLevelEnabled=false;
          RevUpperLevelEnabled=false;
          ResetEnabled=false;
          arm.moveInit=1; 
          arm.MoveComplete=false;
          System.out.printf("\nButton 4 pressed.\n");
          System.out.printf("\nButton 2 pressed.\n");

      }
      if(RevMidLevelEnabled==true)  {
        arm.processRevMidLevel();
      }

      //  Reverse upper position
      if((stick.getRawButtonPressed(4)==true) &&
        (stick.getRawButtonPressed(3)==true) &&
        (arm.MoveComplete==true))  {

          MidLevelEnabled=false;
          LowLevelEnabled=false;
          UpperLevelEnabled=false;
          RevMidLevelEnabled=false;
          RevLowLevelEnabled=false;
          RevUpperLevelEnabled=true;
          ResetEnabled=false;
          arm.moveInit=1; 
          arm.MoveComplete=false;
          System.out.printf("\nButton 4 pressed.\n");
          System.out.printf("\nButton 3 pressed.\n");

      }
      if(RevUpperLevelEnabled==true)  {
        arm.processRevUpperLevel();
      }

  }

}


/////////////////////////////////////////////////////////////////
//  Function:
/////////////////////////////////////////////////////////////////
//
//  Purpose:
//
//  Arguments:
//
//  Returns:
//
//  Remarks:
//
/////////////////////////////////////////////////////////////////
