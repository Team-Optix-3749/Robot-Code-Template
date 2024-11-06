package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.utils.MiscConstants;
import frc.robot.utils.UtilityFunctions;
import frc.robot.utils.MiscConstants.*;
import java.sql.Driver;
import java.util.function.Supplier;

/***
 * @author Noah Simon
 * @author Raadwan Masum
 * @author Rohin Sood
 *         Default command to control the SwervedriveSubsystem with joysticks
 */

public class SwerveDefaultCommand extends Command {

  private final Supplier<Double> xSpdFunction, ySpdFunction, xTurningSpdFunction;

  public SwerveDefaultCommand(
      Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdFunction,
      Supplier<Double> xTurningSpdFunction) {
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.xTurningSpdFunction = xTurningSpdFunction;

    super.addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // controllers are weird in what's positive, so we flip these
    double xMagnitude = -xSpdFunction.get();
    double yMagnitude = -ySpdFunction.get();
    double turningSpeed = -xTurningSpdFunction.get();

    // one combined magnitutde
    double linearMagnitude = Math.hypot(xMagnitude, yMagnitude);
    // one combined direction
    Rotation2d linearDirection = new Rotation2d(xMagnitude, yMagnitude);

    // deadbands
    // is always postive
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, ControllerConstants.deadband);
    // can be negative
    turningSpeed = Math.abs(turningSpeed) > ControllerConstants.deadband
        ? turningSpeed
        : 0.0;

    // squaring the inputs for smoother driving at low speeds
    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
    turningSpeed = Math.copySign(turningSpeed * turningSpeed, turningSpeed);

    double driveSpeedMPS = linearMagnitude * Robot.swerve.getMaxDriveSpeed();

    turningSpeed = turningSpeed * Robot.swerve.getMaxAngularSpeed();

    // Calcaulate new linear components
    double xSpeed = driveSpeedMPS * Math.cos(linearDirection.getRadians());
    double ySpeed = driveSpeedMPS * Math.sin(linearDirection.getRadians());
    ChassisSpeeds chassisSpeeds;

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        UtilityFunctions.isRedAlliance() ? ySpeed : -ySpeed,
        UtilityFunctions.isRedAlliance() ? xSpeed : -xSpeed,
        turningSpeed,
        Robot.swerve.getRotation2d());

    // set chassis speeds
    Robot.swerve.setChassisSpeeds(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
