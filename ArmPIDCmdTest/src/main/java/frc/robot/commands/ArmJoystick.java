package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmJoystick extends CommandBase {
    private final Arm arm;
    private final DoubleSupplier setUpperMotor, setLowerMotor, setLowerMotor2;

    public ArmJoystick(Arm arm, DoubleSupplier setUpperMotor, DoubleSupplier setLowerMotor,
            DoubleSupplier setLowerMotor2) {
        this.arm = arm;
        this.setUpperMotor = setUpperMotor;
        this.setLowerMotor = setLowerMotor;
        this.setLowerMotor2 = setLowerMotor2;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double upperPower = -setUpperMotor.getAsDouble();
        double lowerPower = (-setLowerMotor.getAsDouble() - setLowerMotor2.getAsDouble());

        arm.setUpperMotor(upperPower);
        arm.setLowerMotor(lowerPower);

        // arm.toggleClaw(clawPos);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setUpperMotor(0);
        arm.setLowerMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
