package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForwardCmd extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final double distance;
    private double encoderSetpoint;

    public DriveForwardCmd(DriveSubsystem driveSubsystem, double distance) {
        this.driveSubsystem = driveSubsystem;
        this.distance = distance;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("\nDriveForwardCmd started.\n");

        encoderSetpoint = driveSubsystem.getDriveEncoder() + distance;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveSubsystem.setDriveMotor(0.5);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setDriveMotor(0);


        System.out.println("\nDriveForwardCmd ended.\n");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (driveSubsystem.getDriveEncoder() > encoderSetpoint){
            return true;
        }
        else {
            return false;
        }
    }
}
