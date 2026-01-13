package frc.robot.config;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig.RobotType;
import frc.robot.utils.MiscUtils;

public class ExampleElevatorConfig {
    public static class ElevatorSpecs {
        public static final double GEARING = 1.0 / 4.0 / 4.0; // 16:1 stage followed by another 16:1 stage
        public static final Mass CARRIAGE_MASS = Pounds.of(54);
        // 22 TOOTH, 1/4 in pitch, divide by 2pi to go from circumfrence to radius
        public static final Distance DRUM_RADIUS = Inches.of(4);

        public static final Translation2d MOUNT_OFFSET = new Translation2d(0, Units.inchesToMeters(3));
        public static final Distance MIN_HEIGHT = Meters.of(0);
        public static final Distance MAX_HEIGHT = Feet.of(6);
        public static final Distance STARTING_HEIGHT = Meters.of(0);

        public static boolean IS_INVERTED = false;

        public static final boolean SIMULATE_GRAVITY = true;
    }

    public static final double stateMarginOfError = 0.1;

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
            public LinearVelocity VELOCITY;
            public LinearAcceleration ACCEL;
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
                VELOCITY = MetersPerSecond.of(1.415);
                ACCEL = MetersPerSecondPerSecond.of(4.1);
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
                VELOCITY = MetersPerSecond.of(1.415);
                ACCEL = MetersPerSecondPerSecond.of(4.1);
            }
        };

        public static ControlConfig CONTROL_CONFIG = (MiscUtils.getRobotType() == RobotType.SIM) ? SIM : REAL;
    }

    /***
     * states for the different possible heights the elevator would need to go to
     */
    public enum ElevatorStates {
        STOW(Units.inchesToMeters(0)),
        POSITION_1(Units.feetToMeters(2.5)),
        POSITION_2(Units.feetToMeters(4)),
        MAX(Units.feetToMeters(6)),
        STOPPED(-1);

        /*** the height OFF THE GROUND */
        public Distance height;

        private ElevatorStates(double heightM) {
            this.height = Meters.of(heightM).minus(Meters.of(ElevatorSpecs.MOUNT_OFFSET.getY()));
        }
    }
}