package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.utils.MiscConstants.ControllerConstants;
import frc.robot.utils.UtilityFunctions;

public class DriveStraight extends Command {

    public DriveStraight() {
        super.addRequirements(Robot.swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {


        // one combined magnitutde
        double linearMagnitude = 1;
        // one combined direction
        Rotation2d linearDirection = new Rotation2d(0, 1);

        // deadbands
        // is always postive
        linearMagnitude = MathUtil.applyDeadband(linearMagnitude, ControllerConstants.deadband);
        // can be negative
        double turningMagnitude = 0;

        // squaring the inputs for smoother driving at low speeds
        linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
        turningMagnitude = Math.copySign(turningMagnitude * turningMagnitude, turningMagnitude);

        double driveSpeedMPS = linearMagnitude * Robot.swerve.getMaxDriveSpeed();
        double turningSpeedRadPerSecond = turningMagnitude * Robot.swerve.getMaxAngularSpeed();

        // Calcaulate new linear components
        double xSpeed = driveSpeedMPS * Math.cos(linearDirection.getRadians());
        double ySpeed = driveSpeedMPS * Math.sin(linearDirection.getRadians());
        ChassisSpeeds chassisSpeeds;

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                UtilityFunctions.isRedAlliance() ? ySpeed : -ySpeed,
                UtilityFunctions.isRedAlliance() ? xSpeed : -xSpeed,
                turningSpeedRadPerSecond,
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
