// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class SwerveDriveCmd extends CommandBase {
  //@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final DriveSubsystem driveSubsystem;
  private final Supplier<Double> setDriveMotor, setTurnMotor;

  public SwerveDriveCmd(DriveSubsystem driveSubsystem,
      Supplier<Double> setDriveMotor, Supplier<Double> setTurnMotor) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    this.driveSubsystem = driveSubsystem;
    this.setDriveMotor = setDriveMotor;
    this.setTurnMotor = setTurnMotor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("\nSwerveDriveCmd started.\n");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double realTimeSpeed = setDriveMotor.get();
    double realtimeTurn = setTurnMotor.get();

    driveSubsystem.setDriveMotor(realTimeSpeed);
    driveSubsystem.setTurnMotor(realtimeTurn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setDriveMotor(0);
    driveSubsystem.setTurnMotor(0);
    System.out.println("\nSwerveDriveCmd ended.\n");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
