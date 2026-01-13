package frc.robot.subsystems.ExampleElevator;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.config.ExampleElevatorConfig;

public interface ElevatorIO {
    /**
     * Contains all the data for our elevator
     * 
     * @AutoLog This annotation will automatically log all fields of this class with
     *          Akit.
     */
    @AutoLog
    public class ElevatorData {
        public Distance height = ExampleElevatorConfig.ElevatorSpecs.MOUNT_OFFSET.getMeasureY();
        public LinearVelocity velocity = MetersPerSecond.of(0);
        public LinearAcceleration accel = MetersPerSecondPerSecond.of(0);
        public double leftCurrentAmps = 0;
        public double rightCurrentAmps = 0;
        public double leftAppliedVolts = 0;
        public double rightAppliedVolts = 0;
    }

    /**
     * Set the voltage applied to the elevator motors
     * 
     * @param volts
     */
    public void setVoltage(double volts);

    /**
     * Sets the {@link IdleMode} for the elevator motors
     * 
     * @param idleMode
     */
    public void setMotorIdleMode(IdleMode idleMode);

    /**
     * will be periodically polled from our {@link ExampleElevator} class to update
     * the
     * {@link ElevatorData} class with the latest data
     */
    public void updateData();
}