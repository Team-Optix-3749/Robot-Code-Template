package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

/**
 * Simualtion implementatation for a gyroscope
 * 
 * @author Noah Simon
 * 
 */
public class GyroSim implements GyroIO {
  private Rotation2d yaw = new Rotation2d(0);

  private double prevTimestamp = -1.0;

  @Override
  public void update(GyroData moduleData) {
    double deltaT = 0.02;
    double currTimestamp = Timer.getTimestamp();

    if (prevTimestamp > 0.0) {
      deltaT = currTimestamp - prevTimestamp;
    }
    prevTimestamp = currTimestamp;

    double angleDiffRad = Robot.swerve.getChassisSpeeds().omegaRadiansPerSecond * deltaT;
    yaw = yaw.plus(Rotation2d.fromRadians(angleDiffRad));

    data.orientation = new Rotation3d(0, 0, yaw.getRadians());
    moduleData = data;
  }

  @Override
  public void reset() {
    data.orientation = new Rotation3d(0, 0, 0);
  }
}
