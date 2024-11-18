package frc.robot.subsystems.swerve.real;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.GyroIO;

/**
 * NavX2 gyroscope implementation
 * 
 * @author Noah Simon
 * 
 */
public class NavX2Gyro implements GyroIO {

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    public NavX2Gyro() {
        new Thread(() -> {
            try {
                while (gyro.isCalibrating()) {

                }
                gyro.reset();

            } catch (Exception e) {
            }
        }).start();
    }

    @Override
    public void updateData(GyroData data) {

        data.isCalibrating = gyro.isCalibrating();
        data.isConnected = gyro.isConnected();

        if (data.isConnected && !data.isCalibrating) {
            // negative to make it CCP
            data.yawDeg = -gyro.getYaw();
            data.pitchDeg = gyro.getPitch();
            data.rollDeg = gyro.getRoll();
        } else {
            double angleDiffRad = Robot.swerve.getChassisSpeeds().omegaRadiansPerSecond * 0.02;
            data.yawDeg += (Units.radiansToDegrees(angleDiffRad) + 360) % 360;
        }
    }

    @Override
    public void resetGyro() {
        gyro.reset();
    }
}
