package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToHeadingCmd extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final PIDController pidController;

    private final double speed;
    private final double angle;

    public TurnToHeadingCmd(DriveSubsystem driveSubsystem, double speed, double angle) {
        this.driveSubsystem = driveSubsystem;

        this.pidController = new PIDController(3, 0, 0.8);
        this.pidController.setSetpoint(angle);

        this.speed = speed;
        this.angle = angle;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("\nTurnToHeadingCmd started.\n");

        pidController.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double angle = pidController.calculate(driveSubsystem.getTurnEncoder());
        double speed = pidController.calculate(driveSubsystem.getDriveEncoder());

        driveSubsystem.setTurnMotor(speed);
        driveSubsystem.setDriveMotor(angle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setTurnMotor(0);
        driveSubsystem.setDriveMotor(0);

        System.out.println("\nTurnToHeadingCmd ended.\n");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
