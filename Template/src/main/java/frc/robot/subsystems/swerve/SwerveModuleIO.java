package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

enum SwerveModuleType {
  SIM, SPARK, FLEX
}

/**
 * IO interface for swerve modules. Implementations provide motor, encoder,
 * and control bindings for a single module.
 */
public interface SwerveModuleIO {

  /**
   * Container for all swerve module sensor data and state information.
   */
  @AutoLog
  public class ModuleData {
    /** Module index (0-3) */
    public int index = -1;

    /** Drive motor position in meters */
    public double drivePositionM = 0.0;
    /** Drive motor velocity in meters per second */
    public double driveVelocityMPerSec = 0.0;
    /** Drive motor acceleration in meters per second squared */
    public double driveAccelerationMPerSecSquared = 0.0;
    /** Drive motor desired voltage setpoint */
    public double driveDesiredVolts = 0.0;
    /** Drive motor actual applied voltage */
    public double driveAppliedVolts = 0.0;
    /** Drive motor current draw in amps */
    public double driveCurrentAmps = 0.0;
    /** Drive motor temperature in celsius */
    public double driveTempCelcius = 0.0;

    /** Turn motor position (0-2π radians) */
    public Rotation2d turnPosition = Rotation2d.fromDegrees(0);
    /** Absolute encoder position (0-2π radians) */
    public Rotation2d absoluteEncoderPosition = Rotation2d.fromDegrees(0);
    /** Turn motor velocity in radians per second */
    public double turnVelocityRadPerSec = 0.0;
    /** Turn motor desired voltage setpoint */
    public double turnDesiredVolts = 0.0;
    /** Turn motor actual applied voltage */
    public double turnAppliedVolts = 0.0;
    /** Turn motor current draw in amps */
    public double turnCurrentAmps = 0.0;
    /** Turn motor temperature in celsius */
    public double turnTempCelcius = 0.0;
  }

  /**
   * Runs the drive motor at the specified voltage.
   * 
   * @param volts Voltage to apply (-12 to 12)
   */
  public void setDriveVoltage(double volts);

  /**
   * Runs the turn motor at the specified voltage.
   * 
   * @param volts Voltage to apply (-12 to 12)
   */
  public void setTurnVoltage(double volts);

  /**
   * Enables or disables brake mode on the drive motor.
   * 
   * @param enable True for brake mode, false for coast mode
   */
  public void setDriveBrakeMode(boolean enable);

  /**
   * Enables or disables brake mode on the turn motor.
   * 
   * @param enable True for brake mode, false for coast mode
   */
  public void setTurningBrakeMode(boolean enable);

  /**
   * Sets turn motor position using closed-loop control.
   * 
   * @param position Target angle (0-2π radians)
   */
  public void requestTurnPosition(Rotation2d position);

  /**
   * Sets drive motor velocity using closed-loop control.
   * 
   * @param setpoint Target velocity in meters per second
   */
  public void requestDriveVelocity(double setpoint);

  /**
   * Syncs the relative encoder position with the absolute encoder.
   */
  public void syncEncoderPosition();

  /**
   * Updates sensor data from hardware.
   */
  public void updateData();

}