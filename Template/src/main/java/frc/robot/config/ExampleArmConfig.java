package frc.robot.config;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.utils.MiscUtils;

public class ExampleArmConfig {
    public static class ArmSpecs {
        // this gearing is how many ARM ROTATIONS per MOTOR ROTATION
        // here, we have a 12 tooth sprocket on the motor, driving a 64 tooth
        // sprocket
        // on the arm
        public static final double GEARING = 12.0 / 64.0;
        public static final double END_EFFECTOR_MASS_KG =
                Units.lbsToKilograms(15);
        public static final double ARM_LENGTH_M = Units.inchesToMeters(14);

        public static Translation2d MOUNT_OFFSET =
                new Translation2d(0, Units.inchesToMeters(3));
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(302);
        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d START_ANGLE = Rotation2d.fromDegrees(90);

        public static final boolean SIMULATE_GRAVITY = true;
        public static final boolean IS_INVERTED = false;
    }

    public static class ArmControl {
        public static final MiscUtils.ControlConfigBuilder CONTROL_CONFIG =
                new MiscUtils.ControlConfigBuilder() {
                    {
                        sim()
                                .kG(0.27)
                                .kP(8)
                                .kI(0)
                                .kD(0)
                                .kS(0.16)
                                .kV(7.77)
                                .kA(0.27);
                        real()
                                .kG(0.27)
                                .kP(10)
                                .kI(0)
                                .kD(0)
                                .kS(0.16)
                                .kV(7.77)
                                .kA(0.27);
                    }
                };

        public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(1.415);
        public static final AngularAcceleration MAX_ACCEL = RadiansPerSecondPerSecond.of(4.1);
    }

    public enum ArmStates {
        STOW(Rotation2d.fromDegrees(0)),
        POSITION_1(Rotation2d.fromDegrees(60)),
        POSITION_2(Rotation2d.fromDegrees(250)),
        MAX(ArmSpecs.MAX_ANGLE),
        STOPPED(Rotation2d.fromDegrees(0));

        public Rotation2d angle;

        private ArmStates(Rotation2d s_angle) {
            this.angle = s_angle;
        }
    }
}
