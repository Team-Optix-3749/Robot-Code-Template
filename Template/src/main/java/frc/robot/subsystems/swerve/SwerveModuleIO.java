package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

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
  public class SwerveModuleData {
    public int index = -1;

    public Distance drivePosition = Meters.of(0);
    public LinearVelocity driveVelocity = MetersPerSecond.of(0);
    public LinearAcceleration driveAcceleration = MetersPerSecondPerSecond.of(0);
    public double driveDesiredVolts = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    public double driveTempCelcius = 0.0;

    public Rotation2d turnPosition = Rotation2d.fromDegrees(0);
    public Rotation2d absoluteEncoderPosition = Rotation2d.fromDegrees(0);
    public AngularVelocity turnVelocityRadPerSec = RadiansPerSecond.of(0.0);
    public double turnDesiredVolts = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
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
   * Syncs the relative encoder position with provided offset.
   * @param position The offset to apply to the encoder position
   */
  public void syncEncoderPosition(Rotation2d position);

  /**
   * Updates sensor data from hardware.
   */
  public void updateData();

}