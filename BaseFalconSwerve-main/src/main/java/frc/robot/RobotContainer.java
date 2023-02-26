package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

    /* Driver Buttons */
    private final JoystickButton square = new JoystickButton(stick, PS4Controller.Button.kSquare.value);
    private final JoystickButton cross = new JoystickButton(stick, PS4Controller.Button.kCross.value);
    private final JoystickButton circle = new JoystickButton(stick, PS4Controller.Button.kCircle.value);
    private final JoystickButton triangle = new JoystickButton(stick, PS4Controller.Button.kTriangle.value);
    private final JoystickButton leftBumper = new JoystickButton(stick, PS4Controller.Button.kL1.value);
    private final JoystickButton rightBumper = new JoystickButton(stick, PS4Controller.Button.kR1.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Arm arm = new Arm();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -stick.getRawAxis(leftStickY),
                        () -> -stick.getRawAxis(leftStickX),
                        () -> -stick.getRawAxis(rightStickX),
                        () -> leftBumper.getAsBoolean()));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        square.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));

        cross.onTrue(new InstantCommand(() -> arm.toggleClaw(cross)));

        circle.onTrue(new InstantCommand(() -> s_Swerve.zeroModules()));

        triangle.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        rightBumper.onTrue(new ArmJoystick(arm, s_Swerve, () -> -stick.getRawAxis(leftStickY),
                () -> -stick.getRawAxis(rightStickY)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
