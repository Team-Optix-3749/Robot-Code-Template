package frc.robot.subsystems.swerve.real;

import frc.robot.config.RobotConfig;
import frc.robot.utils.OptixSpark;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.ModuleDataAutoLogged;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveConfig.Drivetrain;
import frc.robot.subsystems.swerve.SwerveConfig.Motor;

public class SwerveModuleSpark implements SwerveModuleIO {
    private final OptixSpark drive;
    private final OptixSpark turn;

    private final CANcoder absoluteEncoder;

    private final ModuleDataAutoLogged data;

    public SwerveModuleSpark(int index, ModuleDataAutoLogged moduleData) {
    data = moduleData;

    data.index = index;

        drive = OptixSpark.ofSparkMax(Motor.driveMotorIds[index]);
        turn = OptixSpark.ofSparkMax(Motor.turnMotorIds[index]);

        drive.setPositionConversionFactor(
                (Math.PI * Drivetrain.wheelDiameterMeters / Motor.driveMotorGearRatio));
        drive.setVelocityConversionFactor(
                (Math.PI * Drivetrain.wheelDiameterMeters / (60 * Motor.driveMotorGearRatio)));
        turn.setPositionConversionFactor((2.0 * Math.PI) / Motor.turnMotorGearRatio);
        turn.setVelocityConversionFactor(2.0 * Math.PI / (Motor.turnMotorGearRatio * 60));

        drive.setSmartCurrentLimit(SwerveConfig.Motor.stallCurrentLimit,
                SwerveConfig.Motor.freeCurrentLimit);
        turn.setSmartCurrentLimit(SwerveConfig.Motor.stallCurrentLimit,
                SwerveConfig.Motor.freeCurrentLimit);

        drive.setIdleMode(IdleMode.kBrake);
        turn.setIdleMode(IdleMode.kBrake);

        turn.setPositionWrapping(-Math.PI, Math.PI);

        drive.apply();
        turn.apply();

        absoluteEncoder = new CANcoder(Motor.absoluteEncoderIds[index]);
        Rotation2d absoluteEncoderOffsetRad = Motor.absoluteEncoderOffsetRad[index];
        turn.requestPosition(
                absoluteEncoder.getPosition().getValueAsDouble() * 2.0 * Math.PI
                        - absoluteEncoderOffsetRad.getRadians());

    absoluteEncoder.optimizeBusUtilization();
    absoluteEncoder.getAbsolutePosition()
        .setUpdateFrequency(RobotConfig.Optimisations.NON_ESSENTIAL_CAN_REFRESH_RATE_HZ);
    }

    @Override
    public void setDriveVoltage(double volts) {
        volts = MathUtil.clamp(volts, -12, 12);

        data.driveDesiredVolts = 0.0;
        drive.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        volts = MathUtil.clamp(volts, -12, 12);
        turn.setVoltage(volts);
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
                .minus(SwerveConfig.Motor.absoluteEncoderOffsetRad[data.index]);

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
