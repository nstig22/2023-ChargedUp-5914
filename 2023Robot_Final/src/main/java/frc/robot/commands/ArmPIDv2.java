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
        this.lowerPidController = new PIDController(0.05, 0.01, 0.001);

        timer = new Timer();

        timer.start();

        upperPidController.setSetpoint(upperSetpoint);
        lowerPidController.setSetpoint(lowerSetpoint);

        upperPidController.setTolerance(4);
        lowerPidController.setTolerance(4);

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        System.out.println("\nArmPIDv2 command started\n");
        timer.reset();
        // timer.start();
        upperPidController.reset();
        lowerPidController.reset();
    }

    @Override
    public void execute() {
        double upperPower = upperPidController.calculate(arm.getUpperMagEncoder());
        double lowerPower = lowerPidController.calculate(arm.getLowerMagEncoder());

        /*
         * if (upperPower > 0.5) {
         * upperPower = 0.5;
         * } else if (upperPower < -0.5) {
         * upperPower = -0.5;
         * }
         * if (lowerPower > 0.5) {
         * lowerPower = 0.5;
         * } else if (lowerPower < -0.5) {
         * lowerPower = -0.5;
         * }
         */

        arm.setUpperMotor(upperPower);

        if (arm.getLowerMagEncoder() >= 0 && arm.getLowerMagEncoder() <= 20 /*|| lowerPidController.getSetpoint() <= 20*/) {
            arm.setLowerMotor(-lowerPower);
        } else {
            arm.setLowerMotor(lowerPower);
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.setUpperMotor(0);
        arm.setLowerMotor(0);
        System.out.println("\nArmPIDv2 command ended\n");
        // System.out.println(upperPower, lowerPower);
    }

    @Override
    public boolean isFinished() {
        /*
         * if (arm.getUpperMagEncoder() >= 120 || arm.getUpperMagEncoder() <= 2) {
         * arm.setUpperMotor(0);
         * arm.setLowerMotor(0);
         * return true;
         * }
         */
        if (upperPidController.atSetpoint() && lowerPidController.atSetpoint()) {
            System.out.println("\nSetpoints hit");
            return true;
        }
        if (arm.getUpperMagEncoder() >= 360 || arm.getUpperMagEncoder() <= 10) {
            System.out.println("\nUpper backwards PID limit hit");
            return true;
        }
        if (arm.getUpperMagEncoder() >= 180 && arm.getUpperMagEncoder() <= 192) {
            System.out.println("\nUpper forwards PID limit hit");
            return true;
        }
        if (arm.getLowerMagEncoder() >= 290 && arm.getLowerMagEncoder() <= 300) {
            System.out.println("\nLower backwards PID limit hit");
            return true;
        }
        if (arm.getLowerMagEncoder() >= 20 && arm.getLowerMagEncoder() <= 50) {
            System.out.println("\nLower forwards PID limit hit");
            return true;
        }
        if (timer.advanceIfElapsed(5)) {
            // timer.reset();
            System.out.println("\nExit on time\n");
            return true;
        } else {
            return false;
        }
    }
}
