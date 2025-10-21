package frc.robot.config;

/**
 * Autonomous tuning values shared across commands. Separating these from the
 * command logic keeps auto routines declarative and easier to retune.
 */
public final class AutoConfig {
    public static final class Drive {
        public static final double KP = 3.0;
        public static final double KD = 0.5;
    }

    public static final class Turn {
        public static final double KP = 4.75;
        public static final double KD = 0.0;
    }
}
