package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.utils.UtilityFunctions;
import frc.robot.utils.MiscConfig.*;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

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
    double controllerX = xSupplier.get();
    double controllerY = -ySupplier.get();
    double controllerTurn = -turnSupplier.get();

    // deadband
    controllerX = UtilityFunctions.signedDeadband(controllerX, Controller.deadbandLX);
    controllerY = UtilityFunctions.signedDeadband(controllerY, Controller.deadbandLY);
    controllerTurn = UtilityFunctions.signedDeadband(controllerTurn, Controller.deadbandRX);

    controllerX = Math.copySign(controllerX, Math.pow(controllerX, Controller.expoFactorTranslate));
    controllerY = Math.copySign(controllerY, Math.pow(controllerY, Controller.expoFactorTranslate));
    controllerTurn = Math.copySign(controllerTurn,
        Math.pow(controllerTurn, Controller.expoFactorRotate));

    // field relative
    double x = controllerX;
    double y = -controllerY;
    double omega = controllerTurn;

    Translation2d movement = new Translation2d(x, y);

    if (movement.getNorm() > 1) {
      movement = movement.div(movement.getNorm());
    }

    // convert to field relative speeds
    // divide by their "contribution" to the total speed
    double xVelocity = movement.getX() * Robot.swerve.getMaxDriveSpeed();
    double yVelocity = movement.getY() * Robot.swerve.getMaxDriveSpeed();
    double turningVelocity = omega * Robot.swerve.getMaxAngularSpeed();

    // flip for red alliance
    if (UtilityFunctions.isRedAlliance()) {
      xVelocity *= -1;
      yVelocity *= -1;
    }

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        yVelocity,
        xVelocity,
        turningVelocity,
        Robot.swerve.getRotation());

    Robot.swerve.driveFieldRelative(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.driveFieldRelative(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
