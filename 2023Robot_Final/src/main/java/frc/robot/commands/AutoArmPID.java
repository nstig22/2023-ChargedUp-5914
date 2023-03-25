package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class AutoArmPID extends CommandBase {
    private final Arm arm;
    private final Timer timer;
    private final PIDController upperPidController;
    private final PIDController lowerPidController;

    public AutoArmPID(Arm arm, double upperSetpoint, double lowerSetpoint) {
        this.arm = arm;
        this.upperPidController = new PIDController(0.01, 0, 0.001);
        this.lowerPidController = new PIDController(0.1, 0, 0.001);

        timer = new Timer();

        timer.start();

        upperPidController.setSetpoint(upperSetpoint);
        lowerPidController.setSetpoint(lowerSetpoint);

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        // System.out.println("\nArmPIDv2 command started\n");
        upperPidController.reset();
        lowerPidController.reset();
    }

    @Override
    public void execute() {
        double upperPower = upperPidController.calculate(arm.getUpperMagEncoder());
        double lowerPower = lowerPidController.calculate(arm.getLowerMagEncoder());

        arm.setUpperMotor(upperPower);

        if (timer.hasElapsed(4)) {
            arm.setLowerMotor(-lowerPower);
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.setUpperMotor(0);
        arm.setLowerMotor(0);
        // System.out.println("\nArmPIDv2 command ended\n");
    }

    @Override
    public boolean isFinished() {
        if (arm.getUpperMagEncoder() >= 120 || arm.getUpperMagEncoder() <= 2) {
            arm.setUpperMotor(0);
            arm.setLowerMotor(0);
            return true;
        }
        if (timer.hasElapsed(7)) {
            arm.toggleClaw();
            Timer.delay(2);
            return true;
        } else {
            return false;
        }
    }
}
