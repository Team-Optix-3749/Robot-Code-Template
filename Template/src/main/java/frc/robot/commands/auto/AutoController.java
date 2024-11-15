package frc.robot.commands.auto;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;

public class AutoController {

    private static PIDController xController = new PIDController(AutoConstants.kPDrive, 0, AutoConstants.kDDrive);
    private static PIDController yController = new PIDController(AutoConstants.kPDrive, 0, AutoConstants.kDDrive);
    private static PIDController turnController = new PIDController(AutoConstants.kPTurn, 0, AutoConstants.kDTurn);

    public static void choreoController(Pose2d curPose, SwerveSample sample) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(
                        xController.calculate(curPose.getX(), sample.x) + sample.vx,
                        yController.calculate(curPose.getY(), sample.y) + sample.vy,
                        turnController.calculate(curPose.getRotation().getRadians(), sample.heading) + sample.omega),
                curPose.getRotation());

        Robot.swerve.setChassisSpeeds(speeds);
    }
}
