/////////////////////////////////////////////////////////////////
//  File:   Robot.java
/////////////////////////////////////////////////////////////////
//
//  Purpose: This file defines the Robot class.  The purpose of 
//  "SimpleRobot", the application name is to debug various
//  components of a new hardware item - SwerveDrive.  This
//  application has test vehicles for both teleOp and autonomous
//  operation.  Autonomous is accomplished via multithreading,
//  i.e., creation of a separate execution thread that is 
//  "time sliced" with the main() thread.  So that instantiation
//  of the SwerveThread class for teloOp and auto are separated,
//  it's either one or the other.  This is accomplished by 
//  declaring these private to Robot, and to SwerveDriveThread
//  classes.  In Robot, the SwerveDrive class is created in
//  teleopInit() so depending on which mode of operation is
//  taking place it's either one or the other.  The functions
//  in SwerveDrive and SwerveDrive thread are different.  When
//  called in teleOp() the function is repeatedly called every
//  20 msec and requires a different logic.  The functions in
//  SwerveDriveThread are called once using while()
//  loops.  Detection of "hang" is accopmplished via timers
//  and the functions will return a specific error code that
//  can be detected and action taken.
//
//  This logic may or may not be used when you create your robot
//  software.  My task is to show you different methods and how
//  to debug and implement various strategies.  1/14/2023 LJB
//
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
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

  // Designate as private - specifically to be used in TeleOp
  // In the thread class there will also be a "private" implementation
  // of the SwerveDrive class
  private SwerveDrive drive;

  double error;

  // autonomous thread parameters
  static SwerveDriveThread test;
  private int init = 1;
  static boolean thread_is_active = false;
  private int auto_update = 0;
  int count;

  // Joystick
  public final Joystick stick = new Joystick(0);

  // Encoder testing
  public DutyCycleEncoder encoder;

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

    // Encoder stuff
    encoder = new DutyCycleEncoder(0);

    encoder.setConnectedFrequencyThreshold(976);
    encoder.setDutyCycleRange(1.0 / 1025, 1025.0 / 1025);

    // Moved this allocation to TeleopInit()
    // drive=new SwerveDrive();
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
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    // Need an 'init' variable. We only launch the thread
    // once. It runs in parallel with autonomousPeriodic()
    // which is called by the timed Robot app aproximately
    // every 20msec.
    if (init == 1) {

      // Start the thread
      test = new SwerveDriveThread("SwerveDriveTest");

      init = 0;

    }
    if (auto_update == 20) {
      // System.out.println("Thread active = " + thread_is_active);
      // System.out.println("SwerveDriveTest" + test.isactive);

      auto_update = 0;
    }
    auto_update++;

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    drive = new SwerveDrive();
    drive.drive_init = 1;

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // Reading encoder
    double position = encoder.getAbsolutePosition();

    count++;

    if (count == 5) {
      System.out.printf("\nEncoder value = %.5f\n", position);
      count = 0;
    }

    // error=drive.rotateRight(45.0);
    // System.out.printf("\nerror = %.3lf\n",error);

    // drive.moveFwd(24.0);

    // drive.moveReverse(36.0);

    // drive.return2Zero();

    drive.falcon_drive.set(ControlMode.PercentOutput, stick.getRawAxis(1));
    drive.falcon_turn.set(ControlMode.PercentOutput, stick.getRawAxis(2));

    if (stick.getRawButton(1) == true) {
      test.isactive = 1;
    }

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
