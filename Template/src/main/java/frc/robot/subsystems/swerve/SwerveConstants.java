package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

public class SwerveConstants {
        public static final class ModuleConstants {

                public static final double wheelDiameterMeters = Units.inchesToMeters(4);
                // 2.04
                // 2.006
                // I think 2 is good lol

                public static final double driveMotorGearRatio = 6.75;
                public static final double turnMotorGearRatio = 12.8;

                private static final double kPTurningReal = 3.75;
                private static final double kDTurningReal = 0;

                private static final double kPDrivingReal = 0.27;
                private static final double kSDrivingReal = 0.26;
                private static final double kVDrivingReal = 2.765;
                private static final double kADrivingReal = 0.0;

                private static final double kPTurningSim = 4;
                private static final double kPDrivingSim = 0.0;
                private static final double kVDrivingSim = 3.19;
                private static final double kSDrivingSim = 0.0;

                public static double kPturning = Robot.isReal()
                                ? kPTurningReal
                                : kPTurningSim;
                public static double kDTurning = Robot.isReal()
                                ? kDTurningReal
                                : 0;
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
                                : 0;

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

                public static final int[] driveMotorPorts = { 3, 5, 7, 9 }; // FL, FR, BL, BR
                public static final int[] turningMotorPorts = { 4, 6, 8, 10 }; // FL, FR, BL, BR
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

                // public static final double[] absoluteEncoderOffsetDeg = { -275, -48, 0, 263
                // };

                public static final int driveMotorStallLimit = 25;
                public static final int driveMotorFreeLimit = 40;
                public static final int turnMotorStallLimit = 25;
                public static final int turnMotorFreeLimit = 40;

                private static final double realMaxSpeedMetersPerSecond = 4.3; // This is our actual top speed
                private static final double realMaxAccelerationMetersPerSecondSquared = 3.3; 

                public static final double teleopMaxSpeedMetersPerSecond = 4.5; // This will send any additional voltage
                                                                                // availible to the motors, making us a
                                                                                // bit faster if we have an extra good
                                                                                // battery charge

                private static final double realMaxAngularSpeedRadiansPerSecond = 6.5;// 12.162 max, needs to be verified
                private static final double realMaxAngularAccelerationRadiansPerSecondSquared = 15.543; // needs to be verified

                private static final double simMaxSpeedMetersPerSecond = 3.707;
                private static final double simMaxAccelerationMetersPerSecondSquared = 2.5;
                private static final double simMaxAngularSpeedRadiansPerSecond = 9.94;
                private static final double simMaxAngularAccelerationRadiansPerSecondSquared = 9;

                private static final double simMaxMotorVoltage = 12.0;
                private static final double realMaxMotorVoltage = 12.0;

                public static final double maxMotorVolts = Robot.isReal()
                                ? DriveConstants.realMaxMotorVoltage
                                : DriveConstants.simMaxMotorVoltage;

                public static final double maxSpeedMetersPerSecond = Robot.isReal()
                                ? DriveConstants.realMaxSpeedMetersPerSecond
                                : DriveConstants.simMaxSpeedMetersPerSecond;

                public static final double maxAngularSpeedRadiansPerSecond = Robot.isReal()
                                ? DriveConstants.realMaxAngularSpeedRadiansPerSecond
                                : DriveConstants.simMaxAngularSpeedRadiansPerSecond;

                public static final double maxAccelerationMetersPerSecondSquared = Robot.isReal()
                                ? DriveConstants.realMaxAccelerationMetersPerSecondSquared
                                : DriveConstants.simMaxAccelerationMetersPerSecondSquared;

                public static final double maxAngularAccelerationRadiansPerSecondSquared = Robot.isReal()
                                ? DriveConstants.realMaxAngularAccelerationRadiansPerSecondSquared
                                : DriveConstants.simMaxAngularAccelerationRadiansPerSecondSquared;

                public static final double toleranceM_Misc = 0.02;
                public static final double toleranceRad_Misc = Math.PI / 750;

                // will eventally be easier to change values from here than poke around through
                // files
        }
}
