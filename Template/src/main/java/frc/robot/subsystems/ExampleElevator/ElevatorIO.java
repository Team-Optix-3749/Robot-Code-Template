package frc.robot.subsystems.ExampleElevator;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public interface ElevatorIO {
    /**
     * Contains all the data for our elevator
     * 
     * @AutoLog This annotation will automatically log all fields of this class with
     *          Akit.
     */
    @AutoLog
    public static class ElevatorData {
        public double positionM = 0;
        public double velocityMPS = 0;
        public double accelMPSS = 0;
        public double leftCurrentAmps = 0;
        public double rightCurrentAmps = 0;
        public double leftAppliedVolts = 0;
        public double rightAppliedVolts = 0;
    }

    /**
     * will be periodically polled from our {@link Elevator} class to update the
     * {@link ElevatorData} class with the latest data
     */
    public default void updateData() {
    };

    /**
     * Set the voltage applied to the elevator motors
     * 
     * @param volts
     */
    public default void setVoltage(double volts) {
    };

    /**
     * Sets the {@link IdleMode} for the elevator motors
     * 
     * @param idleMode
     */
    public default void setMotorIdleMode(IdleMode idleMode) {
    }
}