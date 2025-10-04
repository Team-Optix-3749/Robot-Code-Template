package frc.robot.subsystems.swerve.gyro;

import com.ctre.phoenix6.configs.GyroTrimConfigs;

import edu.wpi.first.math.geometry.Rotation3d;

public class GyroConfig {
    public static Rotation3d mountOrientation = new Rotation3d(90, 0, 0);
    public static GyroTrimConfigs gyroTrimConfigs = new GyroTrimConfigs().withGyroScalarX(-1.21);
}
