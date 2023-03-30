package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;;

public class autoBalanceCmd extends CommandBase {
    private final Swerve s_Swerve;
    private Timer timer;

    //private TalonFX driveMotor = new TalonFX(1);

    SwerveModuleState fwd;
    SwerveModuleState stop;
    SwerveModuleState lock;

    SwerveModuleState[] states = {fwd};

    double speed = 0.5;
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
        
        /*for (SwerveModule mod : s_Swerve.mSwerveMods){
            //mod.setDesiredState(fwd, true);
        }*/

        //s_Swerve.mSwerveMods[0].setDesiredState(fwd, false);

        //driveMotor.set(ControlMode.PercentOutput, 1);

        //s_Swerve.setModuleStates(states);
        /*for (SwerveModule mod : s_Swerve.mSwerveMods){
            //mod.mDriveMotor.set(ControlMode.PercentOutput, 0.5);
        }*/

        s_Swerve.drive(
                new Translation2d(0.5, 0).times(Constants.Swerve.maxSpeed),
                0 * Constants.Swerve.maxAngularVelocity,
                false,
                true);
    }
    

    @Override
    public void end(boolean interrupted) {
        System.out.println("\nAutoBalance command ended\n");
        /*for (SwerveModule mod : s_Swerve.mSwerveMods){
            mod.setDesiredState(lock, true);
        }*/
        /*s_Swerve.drive(
                new Translation2d(0, 0.5).times(Constants.Swerve.maxSpeed),
                0 * Constants.Swerve.maxAngularVelocity,
                false,
                true);*/
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(4)){
            return true;
        }
        else{
            return false;
        }
    }
}
