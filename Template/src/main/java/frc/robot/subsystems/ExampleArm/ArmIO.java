package frc.robot.subsystems.ExampleArm;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Velocity;

public interface ArmIO {
    @AutoLog
    public class ArmData {
        public Rotation2d angle = Rotation2d.fromDegrees(0);
        public double angularVelocityRadPS = 0.0;
        public double angularAccelRadPSS = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public void setVoltage(double volts);
    public void setMotorIdleMode(IdleMode idleMode);
    public void updateData();
}