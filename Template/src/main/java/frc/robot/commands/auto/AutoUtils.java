package frc.robot.commands.auto;

import java.util.Map;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Robot;
import frc.robot.config.RobotConfig.Accuracy;

/**
 * All setup and helper methods for auto routines, including the
 * choreo factory and auto selector
 * 
 * @author Noah Simon
 */
public class AutoUtils {
    // make sure to properly log the robot's setpoints

    private static AutoFactory factory;
    private static AutoChooser chooser;

    private static Map<String, Command> eventMarkerCommands = Map.of(
            "score", new PrintCommand("Scored"));

    /**
     * run all necessary auto setup methods
     */
    public static void initAutoUtils() {
        setupFactory();
        setupChooser();
    }

    private static void setupFactory() {
        factory = new AutoFactory(
                Robot.swerve::getPose, // A function that returns the current robot pose
                Robot.swerve::setOdometry, // A function that resets the current robot pose to the provided Pose2d
                Robot.swerve::driveToSample, // The drive subsystem trajectory follower
                true, // If alliance flipping should be enabled
                Robot.swerve// The drive subsystem
        );

    }

    private static void setupChooser() {

    }

    private static void applyEventMarkers(AutoTrajectory traj, String... markers) {
        for (String marker : markers) {
            if (!eventMarkerCommands.containsKey(marker)) {
                System.out.println("[AutoUtils]: No command found for event marker: " + marker);
                continue;
            }

            traj.atPose(marker, Accuracy.TRANSLATE_TOLERANCE_M, Accuracy.ROTATION_TOLERANCE.getRadians())
                    .onTrue(eventMarkerCommands.get(marker));
        }
    }

    public static AutoChooser getChooser() {
        return chooser;
    }

    public static AutoFactory getAutoFactory() {
        return factory;
    }

    public static Command getSingleTrajectoryCommand(String trajectoryName) {
        AutoRoutine routine = factory.newRoutine(trajectoryName);
        AutoTrajectory traj = routine.trajectory(trajectoryName);

        routine.active().onTrue(Commands.sequence(
                traj.resetOdometry(),
                traj.cmd()));

        Logger.recordOutput("AutoUtils/" + trajectoryName + "/InitialPose", traj.getInitialPose().orElse(Pose2d.kZero));
        return Commands.print("[AutoUtils]: Running - " + trajectoryName).andThen(routine.cmd());
    }
}
