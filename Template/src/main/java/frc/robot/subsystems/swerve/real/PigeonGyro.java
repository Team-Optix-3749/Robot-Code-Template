package frc.robot.subsystems.swerve.real;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.subsystems.swerve.GyroIO;

/**
 * Pigeon 2.0 implementation
 * 
 * @author Noah Simon
 */
public class PigeonGyro implements GyroIO {
    private final Pigeon2 pigeonGyro = new Pigeon2(30);

    public PigeonGyro() {

        try {
            pigeonGyro.reset();
        } catch (Exception e) {
        }
        ;
    }

    @Override
    public void updateData(GyroData data) {
        try {
            data.yawDeg = pigeonGyro.getYaw().getValueAsDouble();
            data.pitchDeg = pigeonGyro.getPitch().getValueAsDouble();
            data.rollDeg = pigeonGyro.getRoll().getValueAsDouble();

        } catch (Exception e) {
        }
        ;
    }

    @Override
    public void resetGyro() {
        pigeonGyro.reset();
    }

}