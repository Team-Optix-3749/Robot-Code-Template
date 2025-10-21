package frc.robot.subsystems.swerve.real;

import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.Can;
import frc.robot.utils.OptixSpark;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.ModuleDataAutoLogged;
import frc.robot.config.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.config.SwerveConfig.Drivetrain;
import frc.robot.config.SwerveConfig.Motor;

public class SwerveModuleSpark implements SwerveModuleIO {
    private final OptixSpark drive;
    private final OptixSpark turn;

    private final CANcoder absoluteEncoder;

    private final ModuleDataAutoLogged data;

    public SwerveModuleSpark(int index, ModuleDataAutoLogged moduleData) {
        data = moduleData;

        data.index = index;

        drive = OptixSpark.ofSparkMax(Can.DRIVE_MOTORS[index]);
        turn = OptixSpark.ofSparkMax(Can.TURN_MOTORS[index]);

        drive.setPositionConversionFactor(
                (Math.PI * Drivetrain.WHEEL_DIA_METERS / Motor.DRIVE_GEARING));
        drive.setVelocityConversionFactor(
                (Math.PI * Drivetrain.WHEEL_DIA_METERS / (60 * Motor.DRIVE_GEARING)));
        turn.setPositionConversionFactor((2.0 * Math.PI) / Motor.TURN_GEARING);
        turn.setVelocityConversionFactor(2.0 * Math.PI / (Motor.TURN_GEARING * 60));

        drive.setSmartCurrentLimit(SwerveConfig.Motor.STALL_CURRENT,
                SwerveConfig.Motor.FREE_CURRENT);
        turn.setSmartCurrentLimit(SwerveConfig.Motor.STALL_CURRENT,
                SwerveConfig.Motor.FREE_CURRENT);

        drive.setIdleMode(IdleMode.kBrake);
        turn.setIdleMode(IdleMode.kBrake);

        turn.setPositionWrapping(-Math.PI, Math.PI);

        drive.apply();
        turn.apply();

        absoluteEncoder = new CANcoder(Can.CANCODERS[index]);
        Rotation2d absoluteEncoderOffsetRad = Motor.CANCODER_OFFSET[index];
        turn.requestPosition(
                absoluteEncoder.getPosition().getValueAsDouble() * 2.0 * Math.PI
                        - absoluteEncoderOffsetRad.getRadians());

        absoluteEncoder.optimizeBusUtilization();
        absoluteEncoder.getAbsolutePosition()
                .setUpdateFrequency(RobotConfig.Optimizations.NON_ESSENTIAL_CAN_REFRESH_HZ);
    }

    @Override
    public void setDriveVoltage(double volts) {
        double clamped = MathUtil.clamp(volts, -12, 12);
        data.driveDesiredVolts = clamped;
        drive.setVoltage(clamped);
    }

    @Override
    public void setTurnVoltage(double volts) {
        double clamped = MathUtil.clamp(volts, -12, 12);
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
