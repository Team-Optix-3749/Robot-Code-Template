package frc.robot.subsystems.ExampleArm.sim;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.config.ExampleArmConfig;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.ExampleArm.ArmDataAutoLogged;
import frc.robot.subsystems.ExampleArm.ArmIO;
import frc.robot.utils.MiscUtils;

/**
 * The reason this class is named weirdly is because "ElevatorSim" is already
 * taken by WPILib to give us the {@link ArmSim} utility class
 */
public class ArmSim implements ArmIO {
    // this has a new MOI parameter, just go ask build for it. It basically
    // describes how hard an arm is to rotate
    private final SingleJointedArmSim armSystemSim = new SingleJointedArmSim(DCMotor.getNEO(1),
            ExampleArmConfig.ArmSpecs.GEARING,
            0.05,
            ExampleArmConfig.ArmSpecs.ARM_LENGTH_M,
            ExampleArmConfig.ArmSpecs.MIN_ANGLE.getRadians(),
            ExampleArmConfig.ArmSpecs.MAX_ANGLE.getRadians(),
            ExampleArmConfig.ArmSpecs.SIMULATE_GRAVITY,
            ExampleArmConfig.ArmSpecs.START_ANGLE.getRadians());

    private ArmDataAutoLogged data;

    static double prevUpdateS = 0;

    public ArmSim(ArmDataAutoLogged elevData) {
        data = elevData;
    }

    @Override
    public void setVoltage(double volts) {
        double clampedVolts = MiscUtils.voltageClamp(volts);
        armSystemSim.setInputVoltage(clampedVolts);
    }

    @Override
    public void setMotorIdleMode(IdleMode idleMode) {
        System.out.println("[ArmSim] setMotorIdleMode called with " + idleMode
                + ", but this has no effect in simulation.");
    }

    @Override
    public void updateData() {
        armSystemSim.update(RobotConfig.GENERAL.NOMINAL_LOOP_TIME_S);

        // when creating Rotation2ds, always use fromRadians or fromDegrees to be
        // explicit
        data.angle = Rotation2d.fromRadians(armSystemSim.getAngleRads());
        data.velocity = RadiansPerSecond.of(armSystemSim.getVelocityRadPerSec());

        // The following data is not useful in simulation:
        data.currentAmps = armSystemSim.getCurrentDrawAmps();
        data.accel = RadiansPerSecondPerSecond.of(0);
    }
}