package frc.robot.subsystems.ExampleElevator.real;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.config.ExampleElevatorConfig.ElevatorSpecs;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.GENERAL;
import frc.robot.subsystems.ExampleElevator.ElevatorDataAutoLogged;
import frc.robot.subsystems.ExampleElevator.ElevatorIO;
import frc.robot.utils.OptixSpark;

/**
 * The reason this class is named weirdly is because "ElevatorSim" is already
 * taken by WPILib to give us the {@link ElevatorSim} utility class
 */
public class ElevatorReal implements ElevatorIO {
    /* data at the top */
    private ElevatorDataAutoLogged data;

    /* motors/outputs next */

    OptixSpark leftMotor = OptixSpark.ofSparkMax(RobotConfig.CAN.ELEVATOR_MOTORS[0]);
    OptixSpark rightMotor = OptixSpark.ofSparkMax(RobotConfig.CAN.ELEVATOR_MOTORS[1]);

    /* anything else after */

    public double previousVelocityMPSS = 0;
    static double prevUpdateS = 0;

    public ElevatorReal(ElevatorDataAutoLogged elevData) {
        /*
         * This works because the REFERENCE to elevData is passed in to the constructor,
         * so any changes we make to "data" will also be reflected in elevData outside
         */
        data = elevData;

        leftMotor
                .setPositionConversionFactor(2 * Math.PI * 2 * ElevatorSpecs.DRUM_RADIUS_M / ElevatorSpecs.GEARING)
                .setSmartCurrentLimit(GENERAL.MED_CURRENT_LIMIT_AMPS)
                .setIdleMode(IdleMode.kBrake)
                .setInverted(ElevatorSpecs.IS_INVERTED);

        // Left and right motors must be synchronized anyway, so why not just simplify it and control one
        leftMotor.apply();
        rightMotor.apply(leftMotor.getConfig());

        rightMotor.follow(leftMotor, true);
    }

    @Override
    public void setVoltage(double volts) {
        double clampedVolts = MathUtil.clamp(volts, -GENERAL.NOMINAL_BUS_VOLTAGE, GENERAL.NOMINAL_BUS_VOLTAGE);

        leftMotor.setVoltage(clampedVolts);
    }

    @Override
    public void updateData() {
        // we used nominal loop time of 20ms for simulation, HOWEVER, in real lift it
        // can vary much more because the processing power on the RIO is much lower. We
        // need to account for that
        double currentTimeS = Timer.getTimestamp();
        double deltaTimeS = prevUpdateS == 0 ? 0.02 : currentTimeS - prevUpdateS;

        // average to try getting a more accurate reading (shouldn't matter much)
        data.positionM = (leftMotor.getPosition() + rightMotor.getPosition()) / 2.0;
        data.velocityMPS = (leftMotor.getVelocity() + rightMotor.getVelocity()) / 2.0;

        data.leftAppliedVolts = leftMotor.getAppliedVolts();
        data.rightAppliedVolts = rightMotor.getAppliedVolts();
        data.leftCurrentAmps = leftMotor.getCurrent();
        data.rightCurrentAmps = rightMotor.getCurrent();

        data.accelMPSS = (data.velocityMPS - previousVelocityMPSS) / deltaTimeS;
        previousVelocityMPSS = data.velocityMPS;
    }
}