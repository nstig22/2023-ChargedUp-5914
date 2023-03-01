package frc.robot.commands;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmPID extends CommandBase {
    private final Arm arm;

    ArmPID(Arm arm) {
        this.arm = arm;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.resetMagEncoders();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
