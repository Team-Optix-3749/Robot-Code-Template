package frc.robot.subsystems.ExampleArm.real;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.config.ExampleElevatorConfig.ElevatorSpecs;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.CurrentLimits;
import frc.robot.config.ExampleArmConfig.ArmSpecs;
import frc.robot.subsystems.ExampleArm.ArmIO;
import frc.robot.subsystems.ExampleArm.ArmDataAutoLogged;
import frc.robot.utils.MiscUtils;
import frc.robot.utils.OptixSpark;

public class ArmReal implements ArmIO {
    /* data at the top */
    private ArmDataAutoLogged data;

    /* motors/outputs next */

    OptixSpark armMotor = OptixSpark.ofSparkMax(RobotConfig.CAN.ARM_MOTOR_ID);

    /* anything else after */

    public AngularVelocity prevVelocity = RadiansPerSecond.of(0);
    static double prevUpdateS = 0;

    public ArmReal(ArmDataAutoLogged armData) {
        data = armData;

        OptixSpark armMotor = OptixSpark.ofSparkMax(RobotConfig.CAN.ARM_MOTOR_ID);
        armMotor
                .setPositionWrapping(-Math.PI, Math.PI)
                .setPositionConversionFactor(ArmSpecs.GEARING * 2.0 * Math.PI)
                .setSmartCurrentLimit(CurrentLimits.DEFAULT_MED)
                .setIdleMode(IdleMode.kBrake)
                .setInverted(ArmSpecs.IS_INVERTED);

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

        data.velocity = RadiansPerSecond.of(armMotor.getVelocity());
        data.accel = data.velocity.minus(prevVelocity).div(Seconds.of(deltaTimeS));

        data.currentAmps = armMotor.getCurrent();
        data.appliedVolts = armMotor.getAppliedVolts();
    }
}