package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    // Motors
    TalonFX upperArmMotor;
    TalonFX lowerArmMotor;

    // Mag encoders
    DutyCycleEncoder upperArmEncoder;
    DutyCycleEncoder lowerArmEncoder;

    // Pneumatics
    Compressor comp;
    PneumaticHub pHub;
    DoubleSolenoid armSolenoid;

    // Constructor
    public Arm() {
        // Motor configs
        upperArmMotor = new TalonFX(Constants.Arm.upperArmMotorID);
        lowerArmMotor = new TalonFX(Constants.Arm.lowerArmMotorID);

        upperArmMotor.setNeutralMode(NeutralMode.Brake);
        lowerArmMotor.setNeutralMode(NeutralMode.Brake);

        upperArmMotor.setInverted(false);
        lowerArmMotor.setInverted(false);

        // Encoders
        upperArmEncoder = new DutyCycleEncoder(Constants.Arm.upperArmEncoderID);
        lowerArmEncoder = new DutyCycleEncoder(Constants.Arm.lowerArmEncoderID);

        // Pneumatics
        comp = new Compressor(Constants.Arm.compID, PneumaticsModuleType.REVPH);

        pHub = new PneumaticHub(Constants.Arm.pHubID);

        armSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

        comp.enableAnalog(115, 120);
    }

    // Set motor values
    public void setMotors(double upperPower, double lowerPower) {
        upperArmMotor.set(ControlMode.PercentOutput, upperPower);
        lowerArmMotor.set(ControlMode.PercentOutput, lowerPower);
    }

    // Get encoder values
    public double getUpperArmEncoder() {
        return upperArmEncoder.getAbsolutePosition();
    }

    public double getLowerArmEncoder() {
        return lowerArmEncoder.getAbsolutePosition();
    }

    // Set pneumatics
    public void toggleClaw(BooleanSupplier state) {
        armSolenoid.toggle();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Upper Arm Encoder Value ", upperArmEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Lower Arm Encoder Value ", lowerArmEncoder.getAbsolutePosition());
    }
}
