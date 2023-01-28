package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    // Motors
    private WPI_TalonFX driveMotor = new WPI_TalonFX(6);
    private WPI_TalonFX turnMotor = new WPI_TalonFX(7);

    // Mag encoder
    private DutyCycleEncoder turnEncoder = new DutyCycleEncoder(1);

    // Gyro
    private ADXRS450_Gyro driveGyro = new ADXRS450_Gyro();

    // Return encoder values
    public double getDriveEncoeer() {
        return driveMotor.getSelectedSensorPosition();
    }

    // Return turn encoder value
    public double getTurnEncoder() {
        return turnEncoder.getAbsolutePosition();
    }

    // Return gyro value
    public double getGyroValue() {
        return driveGyro.getAngle();
    }

    public DriveSubsystem() {
        // turnMotor = new WPI_TalonFX(7);
        turnMotor.setInverted(true);
    }

    public void setDriveMotor(double driveSpeed) {
        driveMotor.set(ControlMode.PercentOutput, driveSpeed);
    }

    public void setTurnMotor(double turnSpeed) {
        turnMotor.set(ControlMode.PercentOutput, turnSpeed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Turn encoder value: ", getTurnEncoder());
        SmartDashboard.putNumber("Drive encoder value: ", getDriveEncoeer());
        SmartDashboard.putNumber("Gyro value: ", getGyroValue());
    }

}
