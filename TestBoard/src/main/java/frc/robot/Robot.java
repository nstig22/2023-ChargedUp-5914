// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  double phi_degrees;
  double theta_degrees;

  RoboticArm arm = new RoboticArm();

  double deadband = 0.1;
  Joystick stick = new Joystick(0);

  TalonFX upperMotor = new TalonFX(2);
  TalonFX lowerMotor = new TalonFX(3);

  Compressor comp = new Compressor(0, PneumaticsModuleType.CTREPCM);

  DoubleSolenoid armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    comp.disable();

    upperMotor.setInverted(false);
    lowerMotor.setInverted(false);

    upperMotor.setNeutralMode(NeutralMode.Brake);
    lowerMotor.setNeutralMode(NeutralMode.Brake);
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    arm.X = 30.0;
    arm.Y = 30.0;

    arm.computeArmAngles(arm.X, arm.Y);

    phi_degrees = arm.Phi * 180.0 / Math.PI;
    theta_degrees = arm.Theta * 180.0 / Math.PI;

    System.out.printf("\nPhi = %.3f  Theta = %.3f degrees", phi_degrees, theta_degrees);
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

    if (arm.lowerRotation_complete == false) {

      arm.rotateLowerArm(theta_degrees);
    }
    if (arm.upperRotation_complete == false) {

      arm.rotateUpperArm(phi_degrees + theta_degrees);
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (stick.getRawButton(6)) {
      comp.enableDigital();
    } else if (stick.getRawButton(5)) {
      comp.disable();
    }

    if (stick.getRawButton(1)) {
      armSolenoid.set(Value.kReverse);
    }
    if (stick.getRawButton(2)) {
      armSolenoid.set(Value.kForward);
    }

    if (stick.getRawButton(4)) {
      arm.computeArmAngles(40, 40);

      if (arm.lowerRotation_complete == false) {

        arm.rotateLowerArm(theta_degrees);
      }
      if (arm.upperRotation_complete == false) {

        arm.rotateUpperArm(phi_degrees + theta_degrees);
      }
    }

    upperMotor.set(ControlMode.PercentOutput, stick.getRawAxis(1));
    lowerMotor.set(ControlMode.PercentOutput, stick.getRawAxis(5));

    /*
     * if (stick.getRawAxis(1) >= deadband){
     * armMotor.set(ControlMode.PercentOutput, stick.getRawAxis(1));
     * }
     * else {
     * armMotor.set(ControlMode.PercentOutput, 0);
     * }
     */

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
