package frc.robot.subsystems.example.real;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.example.SubsystemConstants;
import frc.robot.subsystems.example.SubsystemIO;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for an example subsystem's sparkmax
 * 
 * @author Noah Simon
 */
public class SubsystemSparkMax implements SubsystemIO {

    private CANSparkMax motor = new CANSparkMax(SubsystemConstants.motorId, MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();

    private double inputVolts = 0;
    private double previousVelocity = 0;
    private double velocity = 0;

    public SubsystemSparkMax() {
        motor.setSmartCurrentLimit(30, 50, 1);
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kCoast);
        encoder.setPositionConversionFactor(2 * Math.PI);
        encoder.setVelocityConversionFactor(encoder.getPositionConversionFactor() / 60.0);

    }

    @Override
    public void updateData(SubsystemData data) {
        previousVelocity = velocity;
        velocity = encoder.getVelocity();
        data.positionUnits = encoder.getPosition();
        data.velocityUnits = velocity;
        data.accelerationUnits = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
        data.currentAmps = motor.getOutputCurrent();
        data.inputVolts = inputVolts;
        data.appliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
        data.tempCelcius = motor.getMotorTemperature();

    }

    @Override
    public void setVoltage(double volts) {
        inputVolts = MathUtil.clamp(volts, -12.0, 12.0);
        inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
        motor.setVoltage(inputVolts);

    }
}
