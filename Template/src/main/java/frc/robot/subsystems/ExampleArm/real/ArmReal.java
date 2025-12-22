package frc.robot.subsystems.ExampleArm.real;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.config.ExampleElevatorConfig.ElevatorSpecs;
import frc.robot.config.RobotConfig;
import frc.robot.config.ExampleArmConfig.ArmSpecs;
import frc.robot.config.RobotConfig.GENERAL;
import frc.robot.subsystems.ExampleArm.ArmIO;
import frc.robot.subsystems.ExampleArm.ArmDataAutoLogged;
import frc.robot.utils.MiscUtils;
import frc.robot.utils.OptixSpark;

/**
 * The reason this class is named weirdly is because "ElevatorSim" is already
 * taken by WPILib to give us the {@link ElevatorSim} utility class
 */
public class ArmReal implements ArmIO {
    /* data at the top */
    private ArmDataAutoLogged data;

    /* motors/outputs next */

    OptixSpark armMotor = OptixSpark.ofSparkMax(RobotConfig.CAN.ELEVATOR_MOTORS[0]);

    /* anything else after */

    public double previousVelocityRadPSS = 0;
    static double prevUpdateS = 0;

    public ArmReal(ArmDataAutoLogged elevData) {
        data = elevData;

        // the position is in radians, so we set the wrapping accordingly
        // our "gearing" is how many output ROTATIONS happen per MOTOR ROTATION, so we
        // can multiply the motor rotation by the gearing to get the output rotations.
        // Then multiply that by 2pi to get the motor encoder output in radians.
        // Then apply our -pi to pi code standard for rotations :))
        armMotor
                .setPositionWrapping(-Math.PI, Math.PI)
                .setPositionConversionFactor(ArmSpecs.GEARING * 2.0 * Math.PI)
                .setSmartCurrentLimit(GENERAL.MED_CURRENT_LIMIT_AMPS)
                .setIdleMode(IdleMode.kBrake)
                .setInverted(ElevatorSpecs.IS_INVERTED);

        armMotor.apply();
    }

    @Override
    public void setVoltage(double volts) {
        double clampedVolts = MiscUtils.voltageClamp(volts);

        armMotor.setVoltage(clampedVolts);
    }

    @Override
    public void setMotorIdleMode(IdleMode idleMode) {
        armMotor.setIdleMode(idleMode);
    }

    @Override
    public void updateData() {
        double currentTimeS = Timer.getTimestamp();
        double deltaTimeS = prevUpdateS == 0 ? 0.02 : currentTimeS - prevUpdateS;

        data.angle = Rotation2d.fromRadians(armMotor.getPosition());

        // default units is RPM, but we have the position conversion factor set to
        // radians, so not needed
        data.angularVelocityRadPS = armMotor.getVelocity();
        data.angularAccelRadPSS = (data.angularVelocityRadPS - previousVelocityRadPSS) / deltaTimeS;
        previousVelocityRadPSS = data.angularVelocityRadPS;

        data.currentAmps = armMotor.getCurrent();
        data.appliedVolts = armMotor.getAppliedVolts();
    }
}