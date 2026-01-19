package frc.robot.config;

import com.ctre.phoenix6.configs.GyroTrimConfigs;

import edu.wpi.first.math.geometry.Rotation3d;

public class GyroConfig {
    public static final Rotation3d MOUNT_ORIENTATION = new Rotation3d(90, 0, 0);
    public static final GyroTrimConfigs TRIM_CONFIG = new GyroTrimConfigs().withGyroScalarX(-1.21);
}
