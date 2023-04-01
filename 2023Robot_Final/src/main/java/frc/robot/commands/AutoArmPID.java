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
        this.upperPidController = new PIDController(0.05, 0, 0.001);
        this.lowerPidController = new PIDController(0.1, 0, 0.001);

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
        System.out.println("\nAutoArmPID command started\n");
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
         */

        if (lowerPower > 0.5) {
            lowerPower = 0.5;
        } else if (lowerPower < -0.5) {
            lowerPower = -0.5;
        }

        arm.setUpperMotor(upperPower);

        if (upperPidController.atSetpoint()) {
            arm.setLowerMotor(lowerPower);
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.setUpperMotor(0);
        arm.setLowerMotor(0);
        System.out.println("\nAutoArmPID command ended\n");
        // System.out.println(upperPower, lowerPower);
    }

    @Override
    public boolean isFinished() {
        if (upperPidController.atSetpoint() && lowerPidController.atSetpoint()){
            return true;
        }

        if (arm.getLowerMagEncoder() >= 0 && arm.getLowerMagEncoder() <= 5 || arm.getLowerMagEncoder() >= 315) {
            if (arm.getUpperMagEncoder() <= 255 && arm.getUpperMagEncoder() >= 225) {
                System.out.println("\n6'6'' auto PID limit hit");
                return true;
            }
        }
        /*
         * if (arm.getUpperMagEncoder() <= 220 && arm.getUpperMagEncoder() >= 192)
         * if (arm.getLowerMagEncoder() >= 0
         * && arm.getLowerMagEncoder() <= 20
         * || arm.getLowerMagEncoder() <= 360 && arm.getLowerMagEncoder() >= 315) {
         * upperPower = 0.25;
         * }
         */
        if (arm.getUpperMagEncoder() >= 360 || arm.getUpperMagEncoder() <= 10) {
            System.out.println("\nUpper backwards auto PID limit hit");
            return true;
        }
        if (arm.getUpperMagEncoder() >= 180 && arm.getUpperMagEncoder() <= 192) {
            System.out.println("\nUpper forwards auto PID limit hit");
            return true;
        }
        if (arm.getLowerMagEncoder() >= 290 && arm.getLowerMagEncoder() <= 300) {
            System.out.println("\nLower backwards auto PID limit hit");
            return true;
        }
        if (arm.getLowerMagEncoder() >= 20 && arm.getLowerMagEncoder() <= 50) {
            System.out.println("\nLower forwards auto PID limit hit");
            return true;
        } else {
            return false;
        }
    }
}
