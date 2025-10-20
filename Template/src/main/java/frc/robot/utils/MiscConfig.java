package frc.robot.utils;

import frc.robot.config.RobotConfig;

/**
 * @deprecated Replaced by {@link RobotConfig}. Retained for compatibility
 *             with legacy imports while the codebase transitions to the new
 *             configuration package.
 */
@Deprecated(forRemoval = true)
public final class MiscConfig {

  private MiscConfig() {
    // Legacy bridge
  }

  public enum RobotType {
    REAL(RobotConfig.RobotType.REAL),
    SIM(RobotConfig.RobotType.SIM),
    REPLAY(RobotConfig.RobotType.REPLAY);

    private final RobotConfig.RobotType delegate;

    RobotType(RobotConfig.RobotType delegate) {
      this.delegate = delegate;
    }

    public RobotConfig.RobotType unwrap() {
      return delegate;
    }
  }

  public enum ControlMode {
    BOTH(RobotConfig.ControlMode.BOTH),
    PILOT_ONLY(RobotConfig.ControlMode.PILOT_ONLY),
    OPERATOR_ONLY(RobotConfig.ControlMode.OPERATOR_ONLY),
    SIM(RobotConfig.ControlMode.SIM),
    NONE(RobotConfig.ControlMode.NONE);

    private final RobotConfig.ControlMode delegate;

    ControlMode(RobotConfig.ControlMode delegate) {
      this.delegate = delegate;
    }

    public RobotConfig.ControlMode unwrap() {
      return delegate;
    }
  }

  public static final class CAN {
    public static final int timeoutMs = RobotConfig.Can.TIMEOUT_MS;
    public static final int longTimeoutMs = RobotConfig.Can.LONG_TIMEOUT_MS;
    public static final int configTimeoutMs = RobotConfig.Can.CONFIG_TIMEOUT_MS;
    public static final int PDH_ID = RobotConfig.Can.PDH_ID;

    private CAN() {
    }
  }

  public static final class Sim {
    public static final double loopPeriodSec = RobotConfig.Simulation.LOOP_PERIOD_SEC;

    private Sim() {
    }
  }

  public static final class Controller {
    public static final int pilotPort = RobotConfig.Controller.PILOT_PORT;
    public static final int operatorPort = RobotConfig.Controller.OPERATOR_PORT;

    public static final double deadband = RobotConfig.Controller.DEADBAND;

    public static final double expoFactorTranslate = RobotConfig.Controller.TRANSLATE_EXPO;
    public static final double expoFactorRotate = RobotConfig.Controller.ROTATE_EXPO;

    private Controller() {
    }
  }

  public static final class Optimizations {
    public static final boolean useVision = RobotConfig.Optimisations.USE_VISION;
    public static final int nonEssentialCanRefreshRateHz = RobotConfig.Optimisations.NON_ESSENTIAL_CAN_REFRESH_RATE_HZ;

    private Optimizations() {
    }
  }
}
