package frc.robot.config;

import com.ctre.phoenix6.configs.GyroTrimConfigs;

import edu.wpi.first.math.geometry.Rotation3d;

public class GyroConfig {
    public static Rotation3d MOUNT_ORIENTATION = new Rotation3d(90, 0, 0);
    public static GyroTrimConfigs TRIM_CONFIG = new GyroTrimConfigs().withGyroScalarX(-1.21);
    public static final int PIGEON_GYRO = 40;
}
