package frc.robot.subsystems.swerve;

/**
 * IO interface for gyroscopes
 * 
 * @author Noah Simon
 */
public interface GyroIO {
  public class GyroData {
    public boolean isConnected = false;
    public boolean isCalibrating = false;
    public double yawDeg = 0;
    public double pitchDeg = 0;
    public double rollDeg = 0;
  }

  public default void updateData(GyroData data) {

  }

  public default void resetGyro() {

  }
}
