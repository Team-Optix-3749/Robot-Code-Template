package frc.robot.subsystems.example.real;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.example.SubsystemConstants;
import frc.robot.subsystems.example.SubsystemIO;

public class SubsystemSparkMax implements SubsystemIO {

    private CANSparkMax motor = new CANSparkMax(SubsystemConstants.motorId, MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();

    private double inputVolts = 0;

    public SubsystemSparkMax() {
        motor.setSmartCurrentLimit(30, 50, 1);
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kCoast);
        encoder.setPositionConversionFactor(2 * Math.PI);
        encoder.setVelocityConversionFactor(encoder.getPositionConversionFactor() / 60.0);

    }

    @Override
    public void updateData(SubsystemData data) {

        data.positionUnitsLog.set(encoder.getPosition());
        data.velocityUnitsLog.set(encoder.getVelocity());
        data.currentAmpsLog.set(motor.getOutputCurrent());
        data.inputVoltsLog.set(inputVolts);
        data.appliedVoltsLog.set(motor.getBusVoltage() * motor.getAppliedOutput());
        data.tempCelciusLog.set(motor.getMotorTemperature());

    }

    @Override
    public void setVoltage(double volts) {
        inputVolts = MathUtil.clamp(volts, -12.0, 12.0);
        motor.setVoltage(inputVolts);

    }
}
