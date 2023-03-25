package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmPIDv2 extends CommandBase {
    private final Arm arm;
    private final Timer timer;
    private final PIDController upperPidController;
    private final PIDController lowerPidController;

    public ArmPIDv2(Arm arm, double upperSetpoint, double lowerSetpoint) {
        this.arm = arm;
        this.upperPidController = new PIDController(0.05, 0, 0.001);
        this.lowerPidController = new PIDController(0.1, 0, 0.001);

        timer = new Timer();

        timer.start();

        upperPidController.setSetpoint(upperSetpoint);
        lowerPidController.setSetpoint(lowerSetpoint);

        upperPidController.setTolerance(2);
        lowerPidController.setTolerance(2);

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

        if (upperPower > 0.5) {
            upperPower = 0.5;
        }
        if (lowerPower > 0.5) {
            lowerPower = 0.5;
        }
        if (upperPower < -0.5) {
            upperPower = -0.5;
        }
        if (lowerPower < -0.5) {
            lowerPower = -0.5;
        }

        arm.setUpperMotor(upperPower);
        arm.setLowerMotor(-lowerPower);
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
        if (upperPidController.atSetpoint() == true && lowerPidController.atSetpoint() == true) {
            return true;
        }
        /*
         * if (timer.hasElapsed(4)){
         * return true;
         * }
         */
        else {
            return false;
        }
    }
}
