package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;;

public class autoBalanceCmd extends CommandBase {
    private final Swerve s_Swerve;
    private Timer timer;

    SwerveModuleState fwd;
    SwerveModuleState stop;
    SwerveModuleState lock;

    double speed = 1;
    boolean onChargeStation = false;

    public autoBalanceCmd(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        
        fwd = new SwerveModuleState(speed, Rotation2d.fromDegrees(0));
        stop = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        lock = new SwerveModuleState(0, Rotation2d.fromDegrees(5));
        
        timer = new Timer();
    }

    @Override
    public void initialize() {
        System.out.println("\nAutoBalance command started\n");
        timer.start();
    }

    @Override
    public void execute() {
        if (Math.abs(s_Swerve.ahrs.getPitch()) >= 12){
            speed = 0.1;
            onChargeStation = true;
        }

        if (onChargeStation){
            if (Math.abs(s_Swerve.ahrs.getPitch()) <= 5){
                for (SwerveModule mod : s_Swerve.mSwerveMods){
                    mod.setDesiredState(stop, false);
                    Timer.delay(0.5);
                    mod.setDesiredState(lock, false);
                }
            }
        }
        
        for (SwerveModule mod : s_Swerve.mSwerveMods){
            mod.setDesiredState(fwd, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("\nAutoBalance command ended\n");
        for (SwerveModule mod : s_Swerve.mSwerveMods){
            mod.setDesiredState(lock, true);
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(7)){
            return true;
        }
        else{
            return false;
        }
    }
}
