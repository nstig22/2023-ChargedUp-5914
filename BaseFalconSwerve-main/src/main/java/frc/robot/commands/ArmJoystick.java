package frc.robot.commands;

import java.util.function.Supplier;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmJoystick extends CommandBase {
    private final Arm arm;
    private final Supplier<Double> setUpperMotor, setLowerMotor;

    public ArmJoystick(Arm arm, Supplier<Double> setUpperMotor, Supplier<Double> setLowerMotor) {
        this.arm = arm;
        this.setUpperMotor = setUpperMotor;
        this.setLowerMotor = setLowerMotor;

        addRequirements(arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double upperPower = setUpperMotor.get();
        double lowerPower = setLowerMotor.get();

        arm.setUpperMotor(upperPower);
        arm.setLowerMotor(lowerPower);
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
