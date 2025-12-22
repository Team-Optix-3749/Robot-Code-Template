package frc.robot.subsystems.swerve.gyro;

import com.ctre.phoenix6.Timestamp;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.config.GyroConfig;
import frc.robot.config.RobotConfig;

/**
 * Pigeon 2.0 implementation
 */
public class PigeonGyro implements GyroIO {
    private final Pigeon2 gyro = new Pigeon2(30);

    private final Pigeon2Configuration config = new Pigeon2Configuration();
    private final MountPoseConfigs mountConfig = new MountPoseConfigs();

    public GyroData data = new GyroData();

    public PigeonGyro(GyroData moduleData) {
        data = moduleData;

        Rotation3d mountOrientation = GyroConfig.MOUNT_ORIENTATION;

        mountConfig
                .withMountPoseYaw(mountOrientation.getZ())
                .withMountPosePitch(mountOrientation.getY())
                .withMountPoseRoll(mountOrientation.getX());

        config.withGyroTrim(GyroConfig.TRIM_CONFIG);
        config.withMountPose(mountConfig);

        gyro.reset();
        gyro.getConfigurator().apply(config);

        gyro.optimizeBusUtilization(RobotConfig.OPTIMIZATIONS.ESSENTIAL_CAN_REFRESH_HZ, 0);
    }

    @Override
    public void updateData() {
        /*
         * Pigeon returns orientation as a Rotation3d already :))
         * this also uses quaternion internally as opposed to euler angles, so no gimbal
         * lock issues
         */

        // no timesync
        data.orientation = gyro.getRotation3d();

        // with timesync
        // var sgn_quatW = gyro.getQuatW();
        // var sgn_quatX = gyro.getQuatX();
        // var sgn_quatY = gyro.getQuatY();
        // var sgn_quatZ = gyro.getQuatZ();

        // Timestamp[] timestamps = {
        // sgn_quatW.getTimestamp(),
        // sgn_quatX.getTimestamp(),
        // sgn_quatY.getTimestamp(),
        // sgn_quatZ.getTimestamp()
        // };

        // double timestamp = 0.0;

        // for (Timestamp ts : timestamps) {
        // timestamp += ts.getTime() * 0.25;
        // }

        // Quaternion rotationQuat = new Quaternion(
        // sgn_quatW.getValue(),
        // sgn_quatX.getValue(),
        // sgn_quatY.getValue(),
        // sgn_quatZ.getValue());

        // data.orientation = new Rotation3d(rotationQuat);
        // data.lastUpdateS = timestamp;
    }

    @Override
    public void reset() {
        gyro.reset();
    }

}