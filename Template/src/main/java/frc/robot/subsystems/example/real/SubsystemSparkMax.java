package frc.robot.subsystems.example.real;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.example.ExampleSubsystemConstants;
import frc.robot.subsystems.example.ExampleSubsystemIO;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for an example subsystem's sparkmax
 * 
 * @author Noah Simon
 */
public class SubsystemSparkMax implements ExampleSubsystemIO {

    private SparkMax motor = new SparkMax(ExampleSubsystemConstants.motorId, MotorType.kBrushless);
    private SparkMaxConfig config = new SparkMaxConfig();

    private double inputVolts = 0;
    private double previousVelocity = 0;
    private double velocity = 0;

    public SubsystemSparkMax() {

        config.smartCurrentLimit(30, 50);
        config.encoder.inverted(false);
        config.idleMode(IdleMode.kBrake);
        config.encoder.positionConversionFactor(2 * Math.PI);
        config.encoder.velocityConversionFactor(2 * Math.PI / 60.0);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void updateData(SubsystemData data) {
        previousVelocity = velocity;
        velocity = motor.getEncoder().getVelocity();
        data.positionUnits = motor.getEncoder().getPosition();
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
