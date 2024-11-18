package frc.robot.subsystems.swerve.sim;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.GyroIO;

/**
 * Simualtion implementatation for a gyroscope
 * 
 * @author Noah Simon
 * 
 */
public class GyroSim implements GyroIO {
  private double yaw = 0;
  private double pitch = 0;
  private double roll = 0;

  @Override
  public void updateData(GyroData data) {

    double angleDiffRad = Robot.swerve.getChassisSpeeds().omegaRadiansPerSecond * 0.02;
    Rotation2d currentRotationDiff = Rotation2d.fromRadians(angleDiffRad);

    yaw = (yaw + currentRotationDiff.getDegrees() + 360) % 360;
    data.yawDeg = yaw;
  }

  @Override
  public void resetGyro() {
    GyroData newData = new GyroData();

    // set all gyroData values to newData values
    pitch = newData.pitchDeg;
    roll = newData.rollDeg;
    yaw = newData.yawDeg;
  }
}
