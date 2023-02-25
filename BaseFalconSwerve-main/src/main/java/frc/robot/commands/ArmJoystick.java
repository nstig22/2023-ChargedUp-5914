package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmJoystick extends CommandBase {
    private final Arm arm;
    private final Swerve swerve;
    private final DoubleSupplier setUpperMotor, setLowerMotor;
    private final BooleanSupplier clawPos;

    public ArmJoystick(Arm arm, Swerve swerve, DoubleSupplier setUpperMotor, DoubleSupplier setLowerMotor,
            BooleanSupplier clawPos) {
        this.arm = arm;
        this.swerve = swerve;
        this.setUpperMotor = setUpperMotor;
        this.setLowerMotor = setLowerMotor;
        this.clawPos = clawPos;

        addRequirements(arm, swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double upperPower = setUpperMotor.getAsDouble();
        double lowerPower = setLowerMotor.getAsDouble();

        arm.setMotors(upperPower, lowerPower);

        arm.toggleClaw(clawPos);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setMotors(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
