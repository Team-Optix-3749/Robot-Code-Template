package frc.robot.utils;

import frc.robot.Robot;

/**
 * Constants not specific to any given subsystem or commadn
 * 
 * @author Noah Simon
 */
public class MiscConstants {

  public static enum RobotType {
    REAL,
    SIM,
    REPLAY
  }

  public static final RobotType ROBOT_TYPE = Robot.isReal()
      ? RobotType.REAL
      : RobotType.SIM;

  public static final class SimConstants {
    public static final double loopPeriodSec = 0.02;
  }

  public static final class ControllerConstants {
    public static final double deadbandRX = 0.05;
    public static final double deadbandRY = 0.05;

    public static final double deadbandLX = 0.05;
    public static final double deadbandLY = 0.05;

    public static final double expoFactorTranslate = 1.5;
    public static final double expoFactorRotate = 1.5;
  }
}
