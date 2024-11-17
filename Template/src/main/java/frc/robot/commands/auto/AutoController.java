package frc.robot.commands.auto;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.utils.ShuffleData;
import edu.wpi.first.math.controller.PIDController;

public class AutoController {

    private static PIDController xController = new PIDController(AutoConstants.kPDrive, 0, AutoConstants.kDDrive);
    private static PIDController yController = new PIDController(AutoConstants.kPDrive, 0, AutoConstants.kDDrive);
    private static PIDController turnController = new PIDController(AutoConstants.kPTurn, 0, AutoConstants.kDTurn);

    private static ShuffleData<Double[]> setpointPositionLog = new ShuffleData<Double[]>(
            "Auto",
            "setpoint position",
            new Double[] { 0.0, 0.0, 0.0 });
    private static ShuffleData<Double[]> setpointVelocityLog = new ShuffleData<Double[]>(
            "Auto",
            "setpoint velocity",
            new Double[] { 0.0, 0.0, 0.0 });
    private static ShuffleData<Double[]> setpointAccelerationLog = new ShuffleData<Double[]>(
            "Auto",
            "setpoint acceleration",
            new Double[] { 0.0, 0.0, 0.0 });

    public static void choreoController(Pose2d curPose, SwerveSample sample) {
        logSetpoints(sample);
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(
                        xController.calculate(curPose.getX(), sample.x) + sample.vx,
                        yController.calculate(curPose.getY(), sample.y) + sample.vy,
                        turnController.calculate(curPose.getRotation().getRadians(), sample.heading) + sample.omega),
                curPose.getRotation());

        Robot.swerve.setChassisSpeeds(speeds);
    }

    private static void logSetpoints(SwerveSample sample) {
        Double[] positions = new Double[] { sample.x, sample.y, sample.heading };
        positions[2] = Units.radiansToDegrees(positions[2]);
        setpointPositionLog.set(positions);

        Double[] velocities = new Double[] { sample.vx, sample.vy, sample.omega };
        velocities[2] = Units.radiansToDegrees(velocities[2]);
        setpointVelocityLog.set(velocities);

        Double[] accelerations = new Double[] { sample.ax, sample.ay, sample.alpha };
        accelerations[2] = Units.radiansToDegrees(accelerations[2]);
        setpointAccelerationLog.set(accelerations);
    }
}