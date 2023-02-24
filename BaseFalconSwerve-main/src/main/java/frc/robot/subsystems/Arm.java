package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    PneumaticHub revPH;
    DoubleSolenoid armSolenoid;

    // Constructor
    public Arm() {
        upperArmMotor = new TalonFX(Constants.Arm.upperArmMotorID);
        lowerArmMotor = new TalonFX(Constants.Arm.lowerArmMotorID);

        upperArmEncoder = new DutyCycleEncoder(Constants.Arm.upperArmEncoderID);
        lowerArmEncoder = new DutyCycleEncoder(Constants.Arm.lowerArmEncoderID);

        comp = new Compressor(PneumaticsModuleType.REVPH);

        revPH = new PneumaticHub();

        armSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

        upperArmMotor.setInverted(false);
        lowerArmMotor.setInverted(false);
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
}
