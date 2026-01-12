package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.config.RobotConfig.INPUT;
import frc.robot.utils.MiscUtils;

/**
 * Default driver control for the swerve. Shapes joystick inputs, applies slew
 * rate limiting, and supports robot-relative mode when requested.
 */
public class SwerveDefaultCommand extends Command {
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier turnSupplier;
  // private final BooleanSupplier slowModeSupplier;

  public SwerveDefaultCommand(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier turnSupplier) {
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.turnSupplier = turnSupplier;
    addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // controller x and y is "field relative" we have to make it "driver relative" (basically x is y and y is x)
    // turn is also flipped
    double xInput = applyExpo(ySupplier.getAsDouble(), INPUT.TRANSLATE_EXPO);
    double yInput = applyExpo(xSupplier.getAsDouble(), INPUT.TRANSLATE_EXPO);
    double omegaInput = applyExpo(-turnSupplier.getAsDouble(), INPUT.ROTATE_EXPO);

    double magnitude = Math.hypot(xInput, yInput);
    if (magnitude > 1.0) {
      xInput /= magnitude;
      yInput /= magnitude;
    }

    double speedScale = 1.0;// slowModeSupplier.getAsBoolean() ? Controller.SLOW_MODE_SCALE : 1.0;

    double vx = xInput * Robot.swerve.getMaxDriveSpeed() * speedScale;
    double vy = yInput * Robot.swerve.getMaxDriveSpeed() * speedScale;
    double omega = omegaInput * Robot.swerve.getMaxAngularSpeed() * speedScale;

    double fieldVx = MiscUtils.isRedAlliance() ? vx : -vx;
    double fieldVy = MiscUtils.isRedAlliance() ? vy : -vy;

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        fieldVx,
        fieldVy,
        omega,
        Robot.swerve.getRotation());

    Robot.swerve.driveFieldRelative(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.driveFieldRelative(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private static double applyExpo(double value, double exponent) {
    double limited = MathUtil.clamp(value, -1.0, 1.0);
    double magnitude = Math.pow(Math.abs(limited), exponent);
    return Math.copySign(magnitude, limited);
  }
}
