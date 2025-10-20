package frc.robot.config;

/**
 * Centralized configuration for robot-wide settings that are not tied to a
 * specific subsystem. Grouping these constants makes it easier to reuse them
 * across the codebase while keeping the values in one authoritative location.
 */
public final class RobotConfig {
    private RobotConfig() {
        // Utility class
    }

    /**
     * Distinguishes between deployment targets for behaviour that needs to adapt
     * between real hardware, simulation and log replay.
     */
    public enum RobotType {
        REAL,
        SIM,
        REPLAY
    }

    /**
     * Identifies which physical controllers are connected so the correct control
     * scheme can be selected.
     */
    public enum ControlMode {
        BOTH,
        PILOT_ONLY,
        OPERATOR_ONLY,
        SIM,
        NONE
    }

    /** CAN device IDs and timeouts shared across subsystems. */
    public static final class Can {
        private Can() {
        }

        public static final int TIMEOUT_MS = 250;
        public static final int LONG_TIMEOUT_MS = 1000;
        public static final int CONFIG_TIMEOUT_MS = 5000;
        public static final int PDH_ID = 40;
    }

    /** Simulator specific settings. */
    public static final class Simulation {
        private Simulation() {
        }

        public static final double LOOP_PERIOD_SEC = 0.02;
    }

    /** Configuration for driver/operator controllers. */
    public static final class Controller {
        private Controller() {
        }

        public static final int PILOT_PORT = 0;
        public static final int OPERATOR_PORT = 1;

        public static final double DEADBAND = 0.05;

        public static final double TRANSLATE_EXPO = 1.5;
        public static final double ROTATE_EXPO = 1.5;
    }

    /** Feature toggles and refresh rates that optimise CAN traffic. */
    public static final class Optimisations {
        private Optimisations() {
        }

        public static final boolean USE_VISION = true;
        public static final int NON_ESSENTIAL_CAN_REFRESH_RATE_HZ = 50;
    }
}
