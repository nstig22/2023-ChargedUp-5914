package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.lib.math.Conversions;
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

    // Global inverted variable for switching heading
    Boolean inverted = false;

    // Constructor
    public Arm() {
        // Motor configs
        upperArmMotor = new TalonFX(Constants.Arm.upperArmMotorID);
        lowerArmMotor = new TalonFX(Constants.Arm.lowerArmMotorID);

        upperArmMotor.setNeutralMode(NeutralMode.Brake);
        lowerArmMotor.setNeutralMode(NeutralMode.Brake);

        upperArmMotor.setInverted(false);
        lowerArmMotor.setInverted(false);

        lowerArmMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 50);
        upperArmMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 50);

        // Encoders
        upperArmEncoder = new DutyCycleEncoder(Constants.Arm.upperArmEncoderID);
        upperArmEncoder.setPositionOffset(Constants.Arm.upperArmEncoderOffset / 360);

        lowerArmEncoder = new DutyCycleEncoder(Constants.Arm.lowerArmEncoderID);
        lowerArmEncoder.setPositionOffset(Constants.Arm.lowerArmEncoderOffset / 360);

        // Pneumatics
        comp = new Compressor(Constants.Arm.compID, PneumaticsModuleType.REVPH);

        pHub = new PneumaticHub(Constants.Arm.pHubID);

        armSolenoid = new DoubleSolenoid(Constants.Arm.pHubID, PneumaticsModuleType.REVPH, 4, 5);

        armSolenoid.set(Value.kReverse);

        comp.enableAnalog(110, 120);
    }

    // Set motor values
    public void setUpperMotor(double power) {
        upperArmMotor.set(ControlMode.PercentOutput, power);
    }

    public void setLowerMotor(double power) {
        lowerArmMotor.set(ControlMode.PercentOutput, power);
    }

    // Get falcon encoder values
    public double getUpperFalconEncoder() {
        return upperArmMotor.getSelectedSensorPosition(0);
    }

    public double getLowerFalconEncoder() {
        return lowerArmMotor.getSelectedSensorPosition(0);
    }

    // Reset falcon encoders FIXME

    // Get mag encoder values
    public double getUpperMagEncoder() {
        return ((upperArmEncoder.getAbsolutePosition()) * 360);
    }

    public double getLowerMagEncoder() {
        return ((lowerArmEncoder.getAbsolutePosition()) * 360);
    }

    // Reset falcon encoders FIXME
    public void resetFalconEncoders() {
        upperArmMotor.setSelectedSensorPosition(
                Conversions.degreesToFalcon(getUpperMagEncoder() - Constants.Arm.upperArmEncoderOffset,
                        Constants.Arm.upperGearReduction));
        lowerArmMotor.setSelectedSensorPosition(getLowerMagEncoder());
    }

    // Toggle pneumatics
    public void toggleClaw() {
        armSolenoid.toggle();
        //System.out.println("\nClaw toggled.\n");
    }

    // Switch heading
    public void switchHeading() {
        inverted = !inverted;
        
        upperArmMotor.setInverted(inverted);
        lowerArmMotor.setInverted(inverted);
        

        if (inverted){
            System.out.println("\nInverted == true\n");
        }
        else if (!inverted){
            System.out.println("\nInverted == false\n");
        }
    }

    // TODO Implement limit to avoid extending beyond max height
    public void heightLimit() {

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Upper Arm Encoder Value ", getUpperMagEncoder());
        SmartDashboard.putNumber("Lower Arm Encoder Value ", getLowerMagEncoder());
    }
}
