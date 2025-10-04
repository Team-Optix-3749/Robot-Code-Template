package frc.robot.subsystems.swerve;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
public class SwerveConfig {

        public static final class Control {
                public static final double[] turnPID = new double[] {
                                3, 0, 0 };
                public static final double[] drivePID = new double[] {
                                0.27, 0, 0 };
                // // our FF values
                // public static LoggedTunableNumber kSDriving = new
                // LoggedTunableNumber("swerve/kSDriving",
                // 0.26);
                // public static LoggedTunableNumber kVDriving = new
                // LoggedTunableNumber("swerve/kVDriving",
                // 2.765);
                // public static LoggedTunableNumber kADriving = new
                // LoggedTunableNumber("swerve/kADriving",
                // 0.0);

                // public static LoggedTunableNumber maxVelocity = new
                // LoggedTunableNumber("swerve/maxVelocity",
                // 4.3);
                // public static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(
                // "swerve/maxAcceleration", 3.3);

                public static final double teleopMaxSpeedMetersPerSecond = 4.3;
                public static final double autoMaxSpeedMetersPerSecond = 4.3;

                public static final double maxAngularSpeedRadsPS = 11;
                public static final double maxAngularAccelRadsPSS = 9.0;
        }

        public static final class Motor {
                public static final double driveMotorGearRatio = 6.75;
                public static final double turnMotorGearRatio = 12.8;

                // Module Settings: order is FL, FR, BL, BR
                public static final int[] driveMotorIds = { 3, 5, 7, 9 };
                public static final int[] turnMotorIds = { 4, 6, 8, 10 };
                public static final int[] absoluteEncoderIds = { 11, 12, 13, 14 };

                public static final int stallCurrentLimit = 50;
                public static final int freeCurrentLimit = 40;

                public static final Rotation2d[] absoluteEncoderOffsetRad = {
                                Rotation2d.fromRadians(0.868233),
                                Rotation2d.fromRadians(1.239456),
                                Rotation2d.fromRadians(4.790622),
                                Rotation2d.fromRadians(3.518839)
                };

        }

        public static final class Drivetrain {
                public static final double wheelDiameterMeters = Units.inchesToMeters(4);
                // Distance between right and left wheels
                public static final double trackWidth = Units.inchesToMeters(26.5);
                // Distance between front and back wheels
                public static final double wheelBase = Units.inchesToMeters(20.5);
                public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                                new Translation2d(wheelBase / 2, trackWidth / 2), // front left
                                new Translation2d(wheelBase / 2, -trackWidth / 2), // front right
                                new Translation2d(-wheelBase / 2, trackWidth / 2), // back left
                                new Translation2d(-wheelBase / 2, -trackWidth / 2)); // back right
                public static final Map<Integer, String> moduleNames = Map.of(
                                0, "Front Left",
                                1, "Front Right",
                                2, "Back Left",
                                3, "Back Right");

                // Moment of inertia for simulation (kg*m^2)
                public static final double driveMomentOfInertia = 0.09;
                public static final double turnMomentOfInertia = 0.04;
        }

}
