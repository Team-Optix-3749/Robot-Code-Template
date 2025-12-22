package frc.robot.config;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Centralized configuration for robot-wide settings that are not tied to a
 * specific subsystem. Grouping these constants makes it easier to reuse them
 * across the codebase while keeping the values in one authoritative location.
 */
public final class RobotConfig {
  public enum RobotType {
    REAL,
    SIM,
    REPLAY
  }

  public enum ControlMode {
    BOTH,
    PILOT_ONLY,
    OPERATOR_ONLY,
    SIM,
    NONE
  }

  public static final class Can {
    public static final int TIMEOUT_MS = 250;
    public static final int LONG_TIMEOUT_MS = 1000;
    public static final int CONFIG_TIMEOUT_MS = 5000;
    public static final int PDH_ID = 40;

    // Module Settings: order is FL, FR, BL, BR
    public static final int[] DRIVE_MOTORS = { 3, 5, 7, 9 };
    public static final int[] TURN_MOTORS = { 4, 6, 8, 10 };
    public static final int[] CANCODERS = { 11, 12, 13, 14 };
  }

  /** Simulator specific settings. */
  public static final class Simulation {
    public static final double LOOP_PERIOD_SEC = 0.02;
  }

  /** Configuration for driver/operator controllers. */
  public static final class Controller {
    public static final int PILOT_PORT = 0;
    public static final int OPERATOR_PORT = 1;

    public static final double DEADBAND = 0.05;

    public static final double TRANSLATE_EXPO = 1.5;
    public static final double ROTATE_EXPO = 1.5;
  }

  /** Feature toggles and refresh rates that affect robot performance */
  public static final class Optimizations {
    public static final boolean USE_VISION = true;
    public static final int NON_ESSENTIAL_CAN_REFRESH_HZ = 50;
  }

  public static final class Accuracy {
    public static final double TRANSLATE_TOLERANCE_M = 10000;
    public static final Rotation2d ROTATION_TOLERANCE = Rotation2d.fromDegrees(10000);
  }
}
