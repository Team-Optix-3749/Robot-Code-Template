package frc.robot.subsystems.swerve.gyro;

import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Pigeon 2.0 implementation
 */
public class PigeonGyro implements GyroIO {
    private final Pigeon2 pigeonGyro = new Pigeon2(30);

    private final Pigeon2Configuration config = new Pigeon2Configuration();
    private final MountPoseConfigs mountConfig = new MountPoseConfigs();

    public GyroData data = new GyroData();

    public PigeonGyro() {
        Rotation3d mountOrientation = GyroConfig.mountOrientation;

        mountConfig
                .withMountPoseYaw(mountOrientation.getZ())
                .withMountPosePitch(mountOrientation.getY())
                .withMountPoseRoll(mountOrientation.getX());

        config.withGyroTrim(GyroConfig.gyroTrimConfigs);
        config.withMountPose(mountConfig);

        pigeonGyro.reset();
        pigeonGyro.getConfigurator().apply(config);
    }

    @Override
    public void update(GyroData moduleData) {
        data.orientation = pigeonGyro.getRotation3d();
        moduleData = data;
    }

    @Override
    public void reset() {
        pigeonGyro.reset();
    }

}