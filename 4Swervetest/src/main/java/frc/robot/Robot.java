// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Joystick stick = new Joystick(0);

  // Drive motors
  TalonFX motor1 = new TalonFX(1);
  TalonFX motor3 = new TalonFX(3);
  TalonFX motor5 = new TalonFX(5);
  TalonFX motor7 = new TalonFX(7);

  // Turn motors
  TalonFX motor2 = new TalonFX(2);
  TalonFX motor4 = new TalonFX(4);
  TalonFX motor6 = new TalonFX(6);
  TalonFX motor8 = new TalonFX(8);

  // Turn encoders
  DutyCycleEncoder encoder1 = new DutyCycleEncoder(0);
  DutyCycleEncoder encoder2 = new DutyCycleEncoder(1);
  DutyCycleEncoder encoder3 = new DutyCycleEncoder(2);
  DutyCycleEncoder encoder4 = new DutyCycleEncoder(3);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Drive motors
    motor1.set(ControlMode.PercentOutput, stick.getRawAxis(1));
    motor3.set(ControlMode.PercentOutput, stick.getRawAxis(1));
    motor5.set(ControlMode.PercentOutput, stick.getRawAxis(1));
    motor7.set(ControlMode.PercentOutput, stick.getRawAxis(1));

    // Turn motors
    motor2.set(ControlMode.PercentOutput, stick.getRawAxis(2));
    motor4.set(ControlMode.PercentOutput, stick.getRawAxis(2));
    motor6.set(ControlMode.PercentOutput, stick.getRawAxis(2));
    motor8.set(ControlMode.PercentOutput, stick.getRawAxis(2));

    //Smartdashboard
    SmartDashboard.putNumber("Module 0 encoder ", ((encoder1.getAbsolutePosition() * 360)) - 306);
    SmartDashboard.putNumber("Module 1 encoder ", ((encoder2.getAbsolutePosition() * 360)) - 109);
    SmartDashboard.putNumber("Module 2 encoder ", ((encoder3.getAbsolutePosition() * 360)) - 157);
    SmartDashboard.putNumber("Module 3 encoder ", ((encoder4.getAbsolutePosition() * 360)) - 332);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
