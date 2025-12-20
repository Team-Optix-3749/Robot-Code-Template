package frc.robot.config;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig.RobotType;
import frc.robot.utils.MiscUtils;

public class ExampleElevatorConfig {
    public static class ElevatorSpecs {
        public static final double GEARING = 16 * (24.0 / 22.0);
        public static final double CARRIAGE_MASS_KG = Units.lbsToKilograms(54);
        // 22 TOOTH, 1/4 in pitch, divide by 2pi to go from circumfrence to radius
        public static final double DRUM_RADIUS_M = (Units.inchesToMeters(22.0 / 4.0) / (2 * Math.PI));

        public static Translation2d MOUNT_OFFSET = new Translation2d(0, Units.inchesToMeters(3));
        public static final double MIN_HEIGHT_M = 0;
        public static final double MAX_HEIGHT_M = Units.feetToMeters(6);
        public static final double STARTING_HEIGHT_M = 0;

        public static boolean IS_INVERTED = false;

        public static final boolean SIMULATE_GRAVITY = true;

    }

    public static class ElevatorControl {
        // cleaner utility class to help make switching between real and sim configs
        // easier
        public static class ControlConfig {
            public double kG;
            public double kP;
            public double kI;
            public double kD;
            public double kS;
            public double kV;
            public double kA;
            public double MAX_VELOCITY_MPS;
            public double MAX_ACCEL_MPSS;
        }

        public static ControlConfig SIM = new ControlConfig() {
            {
                kG = 0.27;
                kP = 8;
                kI = 0;
                kD = 0;
                kS = 0.16;
                kV = 7.77;
                kA = 0.27; // 1.72
                MAX_VELOCITY_MPS = 1.415;
                MAX_ACCEL_MPSS = 4.1;
            }
        };

        public static ControlConfig REAL = new ControlConfig() {
            {
                kG = 0.27;
                kP = 10;
                kI = 0;
                kD = 0;
                kS = 0.16;
                kV = 7.77;
                kA = 0.27; // 1.72
                MAX_VELOCITY_MPS = 1.415;
                MAX_ACCEL_MPSS = 4.1;
            }
        };

        public static ControlConfig CONTROL_CONFIG = (MiscUtils.getRobotType() == RobotType.SIM) ? SIM : REAL;
    }

    public enum ElevatorStates {
        // states for the different possible positions the elevator would need to go to
        STOW(Units.inchesToMeters(0)),
        POSITION_1(Units.feetToMeters(2.5)),
        POSITION_2(Units.feetToMeters(4)),
        MAX(Units.feetToMeters(6)),
        STOPPED(-1);

        public Translation2d position;

        private ElevatorStates(double heightM) {
            // Position is relative to the ground, we need relative to mount point
            this.position = new Translation2d(0, heightM - ElevatorSpecs.MOUNT_OFFSET.getY());
        }
    }
}