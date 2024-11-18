package frc.robot.commands.auto;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import edu.wpi.first.math.controller.PIDController;

/**
 * Contains the PID controllers and logic for taking a pose and a swerve sample
 * and having swerve follow it. Potentially should be made non static and placed
 * as an object in the swerve subsystem
 * 
 * @author Noah Simon
 * 
 */
public class AutoController {

    private static PIDController xController = new PIDController(AutoConstants.kPDrive, 0, AutoConstants.kDDrive);
    private static PIDController yController = new PIDController(AutoConstants.kPDrive, 0, AutoConstants.kDDrive);
    private static PIDController turnController = new PIDController(AutoConstants.kPTurn, 0, AutoConstants.kDTurn);

    /**
     * 
     * @param curPose the current pose of the robot in meters, used for measuring
     *                error
     * @param sample  the setpoint sample with position, velocity, acceleration, and
     *                forces
     */
    public static void choreoController(Pose2d curPose, SwerveSample sample) {
        Robot.swerve.logSetpoints(sample);
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(
                        xController.calculate(curPose.getX(), sample.x) + sample.vx,
                        yController.calculate(curPose.getY(), sample.y) + sample.vy,
                        turnController.calculate(curPose.getRotation().getRadians(), sample.heading) + sample.omega),
                curPose.getRotation());

        Robot.swerve.setChassisSpeeds(speeds);
    }

}