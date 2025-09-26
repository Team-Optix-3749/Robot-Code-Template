package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.utils.UtilityFunctions;
import frc.robot.utils.MiscConstants.*;
import java.util.function.Supplier;

/***
 * Default command to control the swerve subsystem with joysticks
 * 
 * @author Noah Simon
 * @author Raadwan Masum
 * @author Rohin Sood
 */

public class SwerveDefaultCommand extends Command {

  private final Supplier<Double> xSupplier, ySupplier, turnSupplier;

  public SwerveDefaultCommand(
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      Supplier<Double> turnSupplier) {
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.turnSupplier = turnSupplier;

    super.addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // controllers are weird in what's positive, so we flip these
    double xMagnitude = xSupplier.get();
    double yMagnitude = ySupplier.get();
    double turningMagnitude = -turnSupplier.get();

    // deadband
    xMagnitude = UtilityFunctions.signedDeadband(xMagnitude, ControllerConstants.deadbandLX);
    yMagnitude = UtilityFunctions.signedDeadband(yMagnitude, ControllerConstants.deadbandLY);
    turningMagnitude = UtilityFunctions.signedDeadband(turningMagnitude, ControllerConstants.deadbandRX);

    xMagnitude = Math.copySign(xMagnitude, Math.pow(xMagnitude, ControllerConstants.expoFactorTranslate));
    yMagnitude = Math.copySign(yMagnitude, Math.pow(yMagnitude, ControllerConstants.expoFactorTranslate));
    turningMagnitude = Math.copySign(turningMagnitude,
        Math.pow(turningMagnitude, ControllerConstants.expoFactorRotate));

    Translation2d movement = new Translation2d(xMagnitude, yMagnitude);

    // If the magnitude is greater than 1, normalize it
    // shouldn't be possible with XBox controller but just in case :))
    if (movement.getNorm() > 1) {
      movement = movement.div(movement.getNorm());
    }

    // convert to field relative speeds
    // divide by their "contribution" to the total speed
    double xVelocity = movement.getX() * SwerveConstants.DriveConstants.maxSpeedMetersPerSecond
        / Math.cos(movement.getAngle().getRadians());
    double yVelocity = movement.getY() * SwerveConstants.DriveConstants.maxSpeedMetersPerSecond
        / Math.cos(movement.getAngle().getRadians());
    double turningVelocity = turningMagnitude * SwerveConstants.DriveConstants.maxAngularSpeedRadiansPerSecond;

    // flip for red alliance
    if (UtilityFunctions.isRedAlliance()) {
      xVelocity *= -1;
      yVelocity *= -1;
    }

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xVelocity,
        yVelocity,
        turningVelocity,
        Robot.swerve.getRotation());

    Robot.swerve.setChassisSpeeds(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
