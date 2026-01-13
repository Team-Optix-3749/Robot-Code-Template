package frc.robot.config;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import java.util.Map;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

/**
 * All constants for the swerve subsystem and swerve modules
 * 
 * @author Noah Simon
 * @author Neel Adem
 * @author Rohin Sood
 * @author Raadwan Masum
 * 
 */
public final class SwerveConfig {

        private SwerveConfig() {
        }

        public static final class Control {
                public static final double MODULE_DRIVE_KS = 0.0;
                public static final double MODULE_DRIVE_KV = 0.0;
                public static final double MODULE_DRIVE_KA = 0.0;
                public static final double[] MODULE_DRIVE_PID = new double[] {
                                12, 0, 0 };
                public static final double[] MODULE_TURN_PID = new double[] {
                                3, 0, 0 };

                public static final double[] TRANSLATE_PID = new double[] {
                                4.5, 0, 0 };
                public static final double[] ROTATE_PID = new double[] {
                                3, 0, 0 };

                // public static LoggedTunableNumber maxVelocity = new
                // LoggedTunableNumber("swerve/maxVelocity",
                // 4.3);
                // public static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(
                // "swerve/maxAcceleration", 3.3);

                public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(4.3);
                public static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(3.3);

                public static final AngularVelocity MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(11);
                public static final AngularAcceleration MAX_ANGULAR_ACCEL = RadiansPerSecondPerSecond.of(9.0);

                public static final Constraints TRANSLATE_CONSTRAINTS = new Constraints(
                                MAX_VELOCITY.in(MetersPerSecond),
                                MAX_ACCEL.in(MetersPerSecondPerSecond));
                public static final Constraints ROTATE_CONSTRAINTS = new Constraints(
                                MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
                                MAX_ANGULAR_ACCEL.in(RadiansPerSecondPerSecond));
        }

        public static final class Motor {
                public static final double DRIVE_GEARING = 6.75;
                public static final double TURN_GEARING = 12.8;

                public static final int STALL_CURRENT = 50;
                public static final int FREE_CURRENT = 40;

                public static final Rotation2d[] CANCODER_OFFSET = {
                                Rotation2d.fromRadians(0.868233),
                                Rotation2d.fromRadians(1.239456),
                                Rotation2d.fromRadians(4.790622),
                                Rotation2d.fromRadians(3.518839)
                };
        }

        public static final class Drivetrain {
                public static final double WHEEL_DIA_METERS = Units.inchesToMeters(4);
                // Distance between right and left wheels
                public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(26);
                // Distance between front and back wheels
                public static final double WHEEL_BASE_METERS = Units.inchesToMeters(26);
                public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                                new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2), // front left
                                new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2), // front right
                                new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2), // back left
                                new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2)); // back right
                public static final Map<Integer, String> MODULE_NAMES = Map.of(
                                0, "Front Left",
                                1, "Front Right",
                                2, "Back Left",
                                3, "Back Right");

                // Moment of inertia for simulation (kg*m^2)
                public static final double TRANSLATE_MOI = 0.025;
                public static final double ROTATE_MOI = 0.004;
        }

        public static final class PoseEstimator {
                public static final Pose2d INITIAL_POSE = new Pose2d(
                                new Translation2d(5.773, 3.963),
                                Rotation2d.fromDegrees(180));

                public static final Vector<N3> STATE_STD_DEVS = VecBuilder.fill(0.045, 0.045, 0.004);
                public static final Vector<N3> VISION_STD_DEVS = VecBuilder.fill(1e-6, 1e-6, 1e-6);
        }

}
