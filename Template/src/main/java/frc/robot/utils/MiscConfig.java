package frc.robot.utils;

import frc.robot.Robot;

/**
 * Constants not specific to any given subsystem or commadn
 * 
 * @author Noah Simon
 */
public class MiscConfig {

  public static RobotType getRobotType() {
    if (Robot.isReal()) {
      return RobotType.REAL;
    } else if (Robot.isSimulation()) {
      return RobotType.SIM;
    } else {
      return RobotType.REPLAY;
    }
  }

  public static enum RobotType {
    REAL,
    SIM,
    REPLAY
  }

  public static RobotType ROBOT_TYPE = getRobotType();

  public static final class CAN {
    public static final int timeoutMs = 250;
    public static final int longTimeoutMs = 1000;
    public static final int configTimeoutMs = 5000;
    public static final int PDH_ID = 40;
  }

  public static final class Sim {
    public static final double loopPeriodSec = 0.02;
  }

  public static final class Controller {
    public static final double deadbandRX = 0.05;
    public static final double deadbandRY = 0.05;

    public static final double deadbandLX = 0.05;
    public static final double deadbandLY = 0.05;

    public static final double expoFactorTranslate = 1.5;
    public static final double expoFactorRotate = 1.5;
  }

  public static final class Optimizations {
    public static final boolean useVision = true;
    public static final int nonEssentialCanRefreshRateHz = 50; // Hz
  }
}
