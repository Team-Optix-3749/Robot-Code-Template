package frc.robot.subsystems.swerve.real;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;

/**
 * Sparkmax implementation for swerve mdoules
 * 
 * @author Noah Simon
 */
public class SwerveModuleSparkMax implements SwerveModuleIO {
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    private CANcoder absoluteEncoder;
    private double absoluteEncoderOffsetRad;

    private double driveAppliedVolts;
    private double turnAppliedVolts;

    private int index;

    public SwerveModuleSparkMax(int index) {
        driveMotor = new CANSparkMax(DriveConstants.driveMotorPorts[index], CANSparkMax.MotorType.kBrushless);
        turnMotor = new CANSparkMax(DriveConstants.turningMotorPorts[index],
                CANSparkMax.MotorType.kBrushless);

        absoluteEncoder = new CANcoder(DriveConstants.absoluteEncoderPorts[index]);
        absoluteEncoderOffsetRad = Units.degreesToRadians(DriveConstants.absoluteEncoderOffsetDeg[index]);

        turnMotor.setInverted(DriveConstants.turningMotorReversed[index]);
        turnMotor.getEncoder().setPositionConversionFactor(1 / ModuleConstants.turnMotorGearRatio * (2 * Math.PI));
        turnMotor.getEncoder()
                .setVelocityConversionFactor(
                        (1 / ModuleConstants.driveMotorGearRatio) * Units.rotationsPerMinuteToRadiansPerSecond(1));

        driveMotor.setInverted(DriveConstants.driveMotorReversed[index]);

        driveMotor.getEncoder().setPositionConversionFactor((1 / ModuleConstants.driveMotorGearRatio) * Math.PI
                * ModuleConstants.wheelDiameterMeters);

        driveMotor.getEncoder().setVelocityConversionFactor(
                (1 / ModuleConstants.driveMotorGearRatio) * Units.rotationsPerMinuteToRadiansPerSecond(1)
                        * (ModuleConstants.wheelDiameterMeters / 2.0));

        driveMotor.setSmartCurrentLimit(DriveConstants.driveMotorStallLimit,
                DriveConstants.driveMotorFreeLimit);
        turnMotor.setSmartCurrentLimit(DriveConstants.turnMotorStallLimit,
                DriveConstants.turnMotorFreeLimit);

        driveMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setIdleMode(IdleMode.kBrake);

        this.index = index;
    };

    @Override
    public void updateData(ModuleData data) {

        driveAppliedVolts = driveMotor.getBusVoltage() * driveMotor.getAppliedOutput();
        data.drivePositionM = getDrivePositionMeters();
        data.driveVelocityMPerSec = getDriveVelocityMetersPerSec();
        data.driveAppliedVolts = driveAppliedVolts;
        data.driveCurrentAmps = Math.abs(driveMotor.getOutputCurrent());
        data.driveTempCelcius = driveMotor.getMotorTemperature();

        turnAppliedVolts = turnMotor.getBusVoltage() * turnMotor.getAppliedOutput();
        data.turnAbsolutePositionRad = getAbsoluteTurningPositionRad();
        data.turnVelocityRadPerSec = getAbsoluteTurninVelocityRadPerSec();
        data.turnAppliedVolts = turnAppliedVolts;
        data.turnCurrentAmps = Math.abs(turnMotor.getOutputCurrent());
        data.turnTempCelcius = turnMotor.getMotorTemperature();
    };

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -DriveConstants.maxMotorVolts,
                DriveConstants.maxMotorVolts);
        driveAppliedVolts = MathUtil.applyDeadband(driveAppliedVolts, 0.05);
        driveMotor.setVoltage(driveAppliedVolts);
    };

    @Override
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -DriveConstants.maxMotorVolts,
                DriveConstants.maxMotorVolts);
        turnAppliedVolts = MathUtil.applyDeadband(turnAppliedVolts, 0.05);

        turnMotor.setVoltage(turnAppliedVolts);
    };

    private double getDrivePositionMeters() {
        return driveMotor.getEncoder().getPosition();
    };

    private double getAbsoluteTurningPositionRad() {
        double pos = Units.rotationsToRadians(absoluteEncoder.getPosition().getValueAsDouble())
                - absoluteEncoderOffsetRad;
        while (pos < 0) {
            pos += 2 * Math.PI;
        }
        while (pos > 2 * Math.PI) {
            pos -= 2 * Math.PI;
        }
        return pos;
    };

    private double getAbsoluteTurninVelocityRadPerSec() {
        return Units.rotationsToRadians(absoluteEncoder.getVelocity().getValueAsDouble());
    };

    private double getDriveVelocityMetersPerSec() {

        return driveMotor.getEncoder().getVelocity();
    };

}