package frc.robot.subsystems.swerve;

/**
 * IO interface for swerve modules. This exists for modules, not
 * the subsystem, as that is what the motors are connected to and
 * interact with
 * 
 * @author Noah Simon
 */
public interface SwerveModuleIO {
  public static class ModuleData {
    public double drivePositionM = 0.0;
    public double driveVelocityMPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    public double driveTempCelcius = 0.0;

    public double turnAbsolutePositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
    public double turnTempCelcius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateData(ModuleData data) {
  }

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {
  }

  /** Run the turn motor at the specified voltage. */
  public default void setTurnVoltage(double volts) {
  }

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {
  }

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurningBrakeMode(boolean enable) {
  }
}