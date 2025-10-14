package frc.robot.subsystems.swerve.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation3d;

/**
 * IO interface for gyroscopes
 */
public interface GyroIO {
  @AutoLog
  public class GyroData {
    public Rotation3d orientation = new Rotation3d(0, 0, 0);
  }

  public default void updateData() {

  }

  public default void reset() {
  }
}
