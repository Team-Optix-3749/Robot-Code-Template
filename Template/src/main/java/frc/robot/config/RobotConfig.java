package frc.robot.config;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

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

  public static final class GENERAL {
    public static final double NOMINAL_LOOP_TIME_S = 0.02;
    public static final double NOMINAL_BUS_VOLTAGE = 12.0;

    public static final int HIGH_CURRENT_LIMIT_AMPS = 60;
    public static final int MED_CURRENT_LIMIT_AMPS = 40;
    public static final int LOW_CURRENT_LIMIT_AMPS = 20;
  }

  public static final class CAN {
    public static final int TIMEOUT_MS = 250;
    public static final int LONG_TIMEOUT_MS = 1000;
    public static final int CONFIG_TIMEOUT_MS = 5000;
    public static final int PDH_ID = 40;

    /** Module Settings: order is FL, FR, BL, BR */
    public static final int[] DRIVE_MOTORS = { 2, 4, 6, 8 };
    public static final int[] TURN_MOTORS = { 3, 5, 7, 9 };
    public static final int[] CANCODERS = { 11, 12, 13, 14 };

    /** Order: Left, Right */
    public static final int[] ELEVATOR_MOTORS = { 20, 21 };
  }

  /** Configuration for driver/operator controllers. */
  public static final class INPUT {
    public static final int PILOT_PORT = 0;
    public static final int OPERATOR_PORT = 1;

    public static final double DEADBAND = 0.05;

    public static final double TRANSLATE_EXPO = 1.5;
    public static final double ROTATE_EXPO = 1.5;
  }

  /** Feature toggles and refresh rates that affect robot performance */
  public static final class OPTIMIZATIONS {
    public static final boolean USE_VISION = true;
    public static final int NON_ESSENTIAL_CAN_REFRESH_HZ = 20;
    public static final int ESSENTIAL_CAN_REFRESH_HZ = 80;
  }

  /** Acceptable tolerances for various robot actions */
  public static final class ACCURACY {
    // default 3 mm/s tolerance for any subsystem movement
    public static final LinearVelocity DEFAULT_MOVEMENT_TOLERANCE = InchesPerSecond.of(0.1);

    public static final Distance DRIVE_TRANSLATE_TOLERANCE = Meters.of(0.02);
    public static final Angle DRIVE_ROTATION_TOLERANCE = Degrees.of(3);

    // 3 cm tolerance for elevator positioning
    public static final Distance ELEVATOR_TOLERANCE = Meters.of(0.025);
  }
}
