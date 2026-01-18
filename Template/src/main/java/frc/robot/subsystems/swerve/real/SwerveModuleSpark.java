package frc.robot.subsystems.swerve.real;

import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.CAN;
import frc.robot.utils.MiscUtils;
import frc.robot.utils.OptixSpark;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveModuleDataAutoLogged;
import frc.robot.config.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.config.SwerveConfig.Drivetrain;
import frc.robot.config.SwerveConfig.Motor;

public class SwerveModuleSpark implements SwerveModuleIO {
    private final OptixSpark drive;
    private final OptixSpark turn;

    private final CANcoder absoluteEncoder;

    private final SwerveModuleDataAutoLogged data;

    public SwerveModuleSpark(int index, SwerveModuleDataAutoLogged moduleData) {
        data = moduleData;

        data.index = index;

        drive = OptixSpark.ofSparkMax(CAN.DRIVE_MOTOR_IDS[index]);
        turn = OptixSpark.ofSparkMax(CAN.TURN_MOTOR_IDS[index]);

        drive
                .setPositionConversionFactor((Math.PI * Drivetrain.WHEEL_DIA_METERS / Motor.DRIVE_GEARING))
                .setVelocityConversionFactor((Math.PI * Drivetrain.WHEEL_DIA_METERS / (60 * Motor.DRIVE_GEARING)))
                .setSmartCurrentLimit(SwerveConfig.Motor.STALL_CURRENT, SwerveConfig.Motor.FREE_CURRENT)
                .setIdleMode(IdleMode.kBrake);
        turn
                .setPositionConversionFactor((2.0 * Math.PI) / Motor.TURN_GEARING)
                .setVelocityConversionFactor(2.0 * Math.PI / (Motor.TURN_GEARING * 60))
                .setSmartCurrentLimit(SwerveConfig.Motor.STALL_CURRENT, SwerveConfig.Motor.FREE_CURRENT)
                .setIdleMode(IdleMode.kBrake);

        drive.apply();
        turn.apply();

        absoluteEncoder = new CANcoder(CAN.CANCODER_IDS[index]);
        Rotation2d absoluteEncoderOffsetRad = Motor.CANCODER_OFFSET[index];
        turn.requestPosition(
                absoluteEncoder.getPosition().getValueAsDouble() * 2.0 * Math.PI
                        - absoluteEncoderOffsetRad.getRadians());

        absoluteEncoder.optimizeBusUtilization();
        absoluteEncoder.getAbsolutePosition()
                .setUpdateFrequency(RobotConfig.OPTIMIZATIONS.NON_ESSENTIAL_CAN_REFRESH_HZ);
    }

    @Override
    public void setDriveVoltage(double volts) {
        double clamped = MiscUtils.voltageClamp(volts);
        data.driveDesiredVolts = clamped;
        drive.setVoltage(clamped);
    }

    @Override
    public void setTurnVoltage(double volts) {
        double clamped = MiscUtils.voltageClamp(volts);
        data.turnDesiredVolts = clamped;
        turn.setVoltage(clamped);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        drive.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setTurningBrakeMode(boolean enable) {
        turn.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void requestDriveVelocity(double setpoint) {
        drive.requestVelocity(setpoint);
    }

    @Override
    public void requestTurnPosition(Rotation2d setpoint) {
        turn.requestPosition(setpoint.getRadians());
    }

    @Override
    public void syncEncoderPosition() {
        Rotation2d absolutePosition = data.absoluteEncoderPosition
                .minus(SwerveConfig.Motor.CANCODER_OFFSET[data.index]);

        turn.setPosition(absolutePosition.getRadians());
        data.turnPosition = absolutePosition;
    }

    @Override
    public void updateData() {
        data.drivePositionM = drive.getPosition();
        data.driveVelocityMPerSec = drive.getVelocity();
        data.driveAppliedVolts = drive.getAppliedVolts();
        data.driveCurrentAmps = drive.getCurrent();
        data.driveTempCelcius = drive.getTemperature();

        data.turnPosition = Rotation2d.fromRadians(turn.getPosition());
        data.absoluteEncoderPosition = Rotation2d.fromRotations(absoluteEncoder.getPosition().getValueAsDouble());
        data.turnVelocityRadPerSec = turn.getVelocity();
        data.turnAppliedVolts = turn.getAppliedVolts();
        data.turnCurrentAmps = turn.getCurrent();
        data.turnTempCelcius = turn.getTemperature();

    };
}
