package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.config.RobotConfig;

/**
 * Simulation implementation for a gyroscope.
 * 
 * @author Noah Simon
 * 
 */
public class GyroSim implements GyroIO {
  private Rotation2d yaw = new Rotation2d(0);

  private final GyroData data;

  public GyroSim(GyroData moduleData) {
    data = moduleData;
  }

  @Override
  public void updateData() {
    double deltaT = RobotConfig.GENERAL.NOMINAL_LOOP_TIME_S;

    double angleDiffRad = Robot.swerve.getChassisSpeeds().omegaRadiansPerSecond * deltaT;
    yaw = yaw.plus(Rotation2d.fromRadians(angleDiffRad));

    data.orientation = new Rotation3d(0, 0, yaw.getRadians());
  }

  @Override
  public void reset() {
    data.orientation = new Rotation3d(0, 0, 0);
  }
}
