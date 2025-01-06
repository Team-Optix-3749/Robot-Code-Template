package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

/**
 * All constants for the swerve subsystem and swerve modules
 * 
 * @author Noah Simon
 * @author Neel Adem
 * @author Rohin Sood
 * @author Raadwan Masum
 * 
 */
public class SwerveConstants {
        public static final class ModuleConstants {

                public static final double wheelDiameterMeters = Units.inchesToMeters(4);

                public static final double driveMotorGearRatio = 6.75;
                public static final double turnMotorGearRatio = 12.8;

                private static final double kPTurningReal = 3.75;
                private static final double kDTurningReal = 0;
                private static final double kPDrivingReal = 0.27;
                private static final double kSDrivingReal = 0.26;
                private static final double kVDrivingReal = 2.765;
                private static final double kADrivingReal = 0.0;

                private static final double kPTurningSim = 4;
                private static final double kDTurningSim = 0;
                private static final double kPDrivingSim = 0.0;
                private static final double kSDrivingSim = 0.0;
                private static final double kVDrivingSim = 12 / DriveConstants.simMaxSpeedMetersPerSecond;
                private static final double kADrivingSim = 1.2;
                // Or have have it be non-constant, (12 - Velocity*kVDrivingSim)/maxAcceleration
                // private static final double kADrivingSim = (12 - 2.94 * kVDrivingSim)
                //                 / DriveConstants.simMaxAccelerationMetersPerSecondSquared;


                // our PID values
                public static double kPturning = Robot.isReal()
                                ? kPTurningReal
                                : kPTurningSim;
                public static double kDTurning = Robot.isReal()
                                ? kDTurningReal
                                : kDTurningSim;
                public static double kPDriving = Robot.isReal()
                                ? kPDrivingReal
                                : kPDrivingSim;
                public static double kSDriving = Robot.isReal()
                                ? kSDrivingReal
                                : kSDrivingSim;
                public static double kVDriving = Robot.isReal()
                                ? kVDrivingReal
                                : kVDrivingSim;
                public static double kADriving = Robot.isReal()
                                ? kADrivingReal
                                : kADrivingSim;
        }

        public static final class DriveConstants {
                // Distance between right and left wheels
                public static final double trackWidth = Units.inchesToMeters(19.5);
                // Distance between front and back wheels
                public static final double wheelBase = Units.inchesToMeters(19.5);
                public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                                new Translation2d(wheelBase / 2, trackWidth / 2), // front left
                                new Translation2d(wheelBase / 2, -trackWidth / 2), // front right
                                new Translation2d(-wheelBase / 2, trackWidth / 2), // back left
                                new Translation2d(-wheelBase / 2, -trackWidth / 2)); // back right

                // Module Settings: order is FL, FR, BL, BR
                public static final int[] driveMotorPorts = { 3, 5, 7, 9 };
                public static final int[] turningMotorPorts = { 4, 6, 8, 10 };
                public static final int[] absoluteEncoderPorts = { 11, 12, 13, 14 };

                public static final boolean[] driveMotorReversed = {
                                true,
                                false,
                                false,
                                false
                };
                public static final boolean[] turningMotorReversed = {
                                false,
                                false,
                                false,
                                false
                };
                public static final boolean[] driveAbsoluteEncoderReversed = {
                                false,
                                false,
                                false,
                                false
                };
                public static final double[] absoluteEncoderOffsetDeg = {
                                26.191,
                                137.594,
                                71.455,
                                186.943
                };

                // current limits
                public static final int driveMotorStallLimit = 25;
                public static final int driveMotorFreeLimit = 50;
                public static final int turnMotorStallLimit = 25;
                public static final int turnMotorFreeLimit = 50;

                // speed
                private static final double realMaxSpeedMetersPerSecond = 4.3; // This is our actual top speed
                private static final double simMaxSpeedMetersPerSecond = 4.55;
                public static final double maxSpeedMetersPerSecond = Robot.isReal()
                                ? DriveConstants.realMaxSpeedMetersPerSecond
                                : DriveConstants.simMaxSpeedMetersPerSecond;
                // acceleration
                private static final double realMaxAccelerationMetersPerSecondSquared = 3.3; // actual top acceleration
                private static final double simMaxAccelerationMetersPerSecondSquared = 6; // not entirely accurate
                public static final double maxAccelerationMetersPerSecondSquared = Robot.isReal()
                                ? DriveConstants.realMaxAccelerationMetersPerSecondSquared
                                : DriveConstants.simMaxAccelerationMetersPerSecondSquared;
                // teleop speed
                public static final double teleopMaxSpeedReduction = 0; // If we can drive a little faster in telop
                                                                        // we may as well
                public static final double teleopMaxSpeedMetersPerSecond = maxSpeedMetersPerSecond
                                * (1 - teleopMaxSpeedReduction);

                // auto speed
                public static final double autoMaxSpeedReduction = 0;
                public static final double autoMaxSpeedMetersPerSecond = maxSpeedMetersPerSecond
                                * (1 - autoMaxSpeedReduction);

                // angular speed
                private static final double realMaxAngularSpeedRadiansPerSecond = 12.162;
                private static final double simMaxAngularSpeedRadiansPerSecond = 12.98;
                public static final double maxAngularSpeedRadiansPerSecond = Robot.isReal()
                                ? DriveConstants.realMaxAngularSpeedRadiansPerSecond
                                : DriveConstants.simMaxAngularSpeedRadiansPerSecond;

                // angular acceleration
                private static final double realMaxAngularAccelerationRadiansPerSecondSquared = 15.543;
                private static final double simMaxAngularAccelerationRadiansPerSecondSquared = 9;
                public static final double maxAngularAccelerationRadiansPerSecondSquared = Robot.isReal()
                                ? DriveConstants.realMaxAngularAccelerationRadiansPerSecondSquared
                                : DriveConstants.simMaxAngularAccelerationRadiansPerSecondSquared;

                // teleop angluar speed
                public static final double teleopMaxAngularSpeedReduction = 0.4;
                public static final double teleopMaxAngularSpeedRadPerSecond = maxAngularSpeedRadiansPerSecond
                                * (1 - teleopMaxAngularSpeedReduction);

                // auto angular speed
                public static final double autoMaxAngularSpeedReduction = 0;
                public static final double autoMaxAngularSpeedRadPerSecond = maxAngularSpeedRadiansPerSecond
                                * (1 - autoMaxAngularSpeedReduction);

                // motor volts
                private static final double simMaxMotorVoltage = 12.0;
                private static final double realMaxMotorVoltage = 12.0;
                public static final double maxMotorVolts = Robot.isReal()
                                ? DriveConstants.realMaxMotorVoltage
                                : DriveConstants.simMaxMotorVoltage;

        }
}
