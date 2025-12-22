package frc.robot.subsystems.ExampleElevator.sim;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.config.ExampleElevatorConfig;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.GENERAL;
import frc.robot.subsystems.ExampleElevator.ElevatorDataAutoLogged;
import frc.robot.subsystems.ExampleElevator.ElevatorIO;
import frc.robot.utils.MiscUtils;

/**
 * The reason this class is named weirdly is because "ElevatorSim" is already
 * taken by WPILib to give us the {@link ElevatorSim} utility class
 */
public class ElevatorSiml implements ElevatorIO {
    private final ElevatorSim elevatorSimSystem = new ElevatorSim(
            DCMotor.getNEO(2),
            ExampleElevatorConfig.ElevatorSpecs.GEARING,
            ExampleElevatorConfig.ElevatorSpecs.CARRIAGE_MASS_KG,
            ExampleElevatorConfig.ElevatorSpecs.DRUM_RADIUS_M,
            ExampleElevatorConfig.ElevatorSpecs.MIN_HEIGHT_M,
            ExampleElevatorConfig.ElevatorSpecs.MAX_HEIGHT_M,
            true,
            ExampleElevatorConfig.ElevatorSpecs.STARTING_HEIGHT_M);

    /**
     * This is our {@link ElevatorData} instance that we will update with data
     * We have to use the "AutoLogged" variation because of the @AutoLog annotation,
     * which will create a new class for the data object
     */
    private ElevatorDataAutoLogged data;

    static double prevUpdateS = 0;

    public ElevatorSiml(ElevatorDataAutoLogged elevData) {
        /*
         * This works because the REFERENCE to elevData is passed in to the constructor,
         * so any changes we make to "data" will also be reflected in elevData outside
         */
        data = elevData;
    }

    @Override
    public void setVoltage(double volts) {
        /*
         * Theoretically, a voltage higher than the max for the motor could be applied,
         * so clamp to make sure that doesn't happen
         * 
         * "Nominal Bus Voltage" is just the expected voltage from the battery (12V)
         * 
         * When using the {@link OptixSpark} class, this can be omitted
         */
        double clampedVolts = MiscUtils.voltageClamp(volts);

        /*
         * One of the perks of using the reference is that we can update the data object
         * outside the updateData method
         */
        data.leftAppliedVolts = clampedVolts;
        data.rightAppliedVolts = clampedVolts;

        elevatorSimSystem.setInputVoltage(clampedVolts);
    }

    @Override
    public void setMotorIdleMode(IdleMode idleMode) {
        System.out.println("[ElevatorSim] setMotorIdleMode called with " + idleMode
                + ". Note: Idle mode has no effect in simulation.");
    }

    @Override
    public void updateData() {
        elevatorSimSystem.update(RobotConfig.GENERAL.NOMINAL_LOOP_TIME_S);

        data.position = new Translation2d(0, elevatorSimSystem.getPositionMeters() +
                ExampleElevatorConfig.ElevatorSpecs.MOUNT_OFFSET.getY());
        data.velocityMPS = elevatorSimSystem.getVelocityMetersPerSecond();

        // The following data is not useful in simulation:
        data.leftCurrentAmps = elevatorSimSystem.getCurrentDrawAmps();
        data.rightCurrentAmps = elevatorSimSystem.getCurrentDrawAmps();
        // this *could* be calculated by using ((currentVelocity -
        // prevVelocity)/deltaTimeS), but once again, not worth it for sim
        data.accelMPSS = 0;
    }
}