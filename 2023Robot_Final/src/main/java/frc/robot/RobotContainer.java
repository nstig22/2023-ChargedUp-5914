package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick stick = new Joystick(0);

    /* Drive Controls */
    private final int leftStickX = PS4Controller.Axis.kLeftX.value;
    private final int leftStickY = PS4Controller.Axis.kLeftY.value;
    private final int rightStickX = PS4Controller.Axis.kRightX.value;
    private final int rightStickY = PS4Controller.Axis.kRightY.value;
    private final int leftTrigger = PS4Controller.Axis.kL2.value;
    private final int rightTrigger = PS4Controller.Axis.kR2.value;

    /* Driver Buttons */
    private final JoystickButton square = new JoystickButton(stick, PS4Controller.Button.kSquare.value);
    private final JoystickButton cross = new JoystickButton(stick, PS4Controller.Button.kCross.value);
    private final JoystickButton circle = new JoystickButton(stick, PS4Controller.Button.kCircle.value);
    private final JoystickButton triangle = new JoystickButton(stick, PS4Controller.Button.kTriangle.value);
    private final JoystickButton leftBumper = new JoystickButton(stick, PS4Controller.Button.kL1.value);
    private final JoystickButton rightBumper = new JoystickButton(stick, PS4Controller.Button.kR1.value);
    private final JoystickButton touchpad = new JoystickButton(stick, PS4Controller.Button.kTouchpad.value);
    private final JoystickButton share = new JoystickButton(stick, PS4Controller.Button.kShare.value);
    private final JoystickButton options = new JoystickButton(stick, PS4Controller.Button.kOptions.value);
    private final JoystickButton ps = new JoystickButton(stick, PS4Controller.Button.kPS.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Arm arm = new Arm();

    /* Autonomous routines */
    private final Command driveFwd = new testAuto(s_Swerve);
    private final Command midCubeExit = new SequentialCommandGroup(new AutoArmPID(arm, 295, 340),
            new InstantCommand(() -> arm.toggleClaw()), new ParallelCommandGroup(new testAuto(s_Swerve)),
            new AutoArmPID(arm, 354, 312));
    private final Command balance = new autoBalanceCmd(s_Swerve);
    private final Command testBalance = new balanceAuto(s_Swerve);
    private final Command highCubeExit = new SequentialCommandGroup(new AutoArmPID(arm, 258, 354),
            new InstantCommand(() -> arm.toggleClaw()),
            new ParallelCommandGroup(new InstantCommand(() -> arm.toggleClaw()), new AutoArmPID(arm, 354, 312),
                    new testAuto(s_Swerve)));
    private final Command highCubeEngage = new SequentialCommandGroup(new AutoArmPID(arm, 258, 354),
            new InstantCommand(() -> arm.toggleClaw()), new ParallelCommandGroup(new AutoArmPID(arm, 354, 312)),
            new engageAuto(s_Swerve));
    private final Command highCube = new AutoArmPID(arm, 258, 354);
    private final Command noAuto = null;

    /* Sendable chooser for autonomous commands */
    SendableChooser<Command> m_Chooser = new SendableChooser<>();

    /** 
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> stick.getRawAxis(leftStickY),
                        () -> stick.getRawAxis(leftStickX),
                        () -> stick.getRawAxis(rightStickX),
                        () -> options.getAsBoolean()));

        arm.setDefaultCommand(
                new ArmJoystick(arm, () -> stick.getRawAxis(rightStickY), () -> stick.getRawAxis(leftTrigger),
                        () -> -stick.getRawAxis(rightTrigger), () -> leftBumper.getAsBoolean(),
                        () -> rightBumper.getAsBoolean()));

        // Configure the button bindings
        configureButtonBindings();

        // Add commands to the autonomous chooser
        // m_Chooser.setDefaultOption("High cube, mobility, & balance", null);
        m_Chooser.addOption("Drive forward", driveFwd);
        m_Chooser.addOption("Mid cube and exit", midCubeExit);
        m_Chooser.addOption("High cube and exit", highCubeExit);
        m_Chooser.addOption("High cube and engage", highCubeEngage);
        m_Chooser.addOption("High cube no drive", highCube);
        m_Chooser.addOption("No auto", noAuto);
        // m_Chooser.addOption("Balance", balance);
        // m_Chooser.addOption("test balance", testBalance);

        SmartDashboard.putData(m_Chooser);
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        // square.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));

        cross.onTrue(new InstantCommand(() -> arm.toggleClaw()));

        // circle.onTrue(new ArmPID(arm, 30, 30));
        // circle.onTrue(new ArmPID(arm, circle, square, rightBumper));
        circle.onTrue(new ArmPIDv2(arm, 354, 312));

        triangle.onTrue(new ArmPIDv2(arm, 273, 338));

        square.onTrue(new ArmPIDv2(arm, 304, 310));

        touchpad.onTrue(new ArmPIDv2(arm, 210, 350));

        ps.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        // touchpad.onTrue(new InstantCommand(() -> arm.switchHeading()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_Chooser.getSelected();
    }
}
