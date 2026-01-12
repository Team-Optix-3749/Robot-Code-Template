package frc.robot.commands.swerve;

import java.io.IOException;
import org.json.simple.parser.ParseException;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.config.SwerveConfig;
import frc.robot.subsystems.swerve.ToPos;

/**
 * The `OnTheFly` command dynamically generates and follows a trajectory
 * from the robot's current position to a designated scoring or movement
 * setpoint.
 * It is responsible for real-time path planning and execution using
 * PathPlanner.
 */
public class OnTheFly extends Command {
    private PathPlannerTrajectory trajectory; // The generated trajectory for movement
    private final Timer timer = new Timer(); // Timer to track trajectory progress
    private final ToPos toPos = new ToPos();
    private final Timer debugTimer = new Timer();
    private final Pose2d pose;

    private Pose2d endpoint = new Pose2d();
    private static boolean currentlyThreading = false; // threads make funky async stuff: this normalizes it

    /**
     * Constructs the OnTheFly command.
     * This command does not require any parameters as it dynamically determines the
     * path.
     */
    public OnTheFly(Pose2d pose) {
        this.pose = pose;
        addRequirements(Robot.swerve);
    }

    public static final double approachPointDistance = 0.75;

    private static Pose2d createApproachPoint(Pose2d pose) {
        Translation2d position = pose.getTranslation();
        Rotation2d heading = pose.getRotation();

        // Calculate an offset distance backward in the direction of the heading
        Translation2d offset = new Translation2d(
                -approachPointDistance * Math.cos(heading.getRadians()),
                -approachPointDistance * Math.sin(heading.getRadians()));

        return new Pose2d(position.plus(offset), heading);
    }

    /**
     * Initializes the trajectory generation process and starts the timer.
     * If no valid path is generated, the command cancels itself.
     */
    @Override
    public void initialize() {
        // System.out.println("epic fail");

        currentlyThreading = true;
        endpoint = pose;

        new Thread(() -> {
            debugTimer.reset();
            debugTimer.start();
            // Create a new dynamic path generator
            PathPlannerPath path = toPos.generateDynamicPath(
                    Robot.swerve.getPose(), // Current robot position
                    createApproachPoint(pose), // Intermediate approach point
                    pose, // Final target position
                    Robot.swerve.getMaxDriveSpeed(), // Max driving speed
                    SwerveConfig.Control.MAX_ACCEL_MPSS, // Max acceleration
                    Robot.swerve.getMaxAngularSpeed(), // Max angular speed
                    SwerveConfig.Control.MAX_ANGULAR_ACCEL_RADSSS // Max angular
                                                                  // acceleration
            );
            // System.out.println("path generation took: " + debugTimer.get());
            debugTimer.stop();

            // If path generation fails, stop the command
            if (path == null) {
                Robot.swerve.setIsOTF(false);
                currentlyThreading = false;
                this.cancel();
                return;
            }

            try {
                // Generate the trajectory using the planned path and current robot state
                trajectory = path.generateTrajectory(
                        Robot.swerve.getChassisSpeeds(), // Current velocity
                        Robot.swerve.getRotation(), // Current rotation
                        RobotConfig.fromGUISettings()); // PathPlanner robot config settings

                var states = trajectory.getStates();
                if (states.size() >= 2) {
                    // Placeholder: Optionally store second-to-last waypoint
                }
            } catch (IOException | ParseException e) {
                // If an error occurs during trajectory generation, stop execution
                e.printStackTrace();
                Robot.swerve.setIsOTF(false);
                currentlyThreading = false;
                this.cancel();
            }
            currentlyThreading = false;
            timer.reset();
            timer.start();
            // System.out.println(debugTimer.get());
        }).start();
    }

    /**
     * Continuously executes the trajectory-following logic.
     * If the trajectory is invalid or the robot is not in OTF mode, the command
     * cancels itself.
     */
    @Override
    public void execute() {
        if (!pose.equals(endpoint)) {
            initialize();
        }
        if (currentlyThreading) {
            return;
        }

        if ((trajectory == null || !Robot.swerve.getIsOTF())) {
            this.cancel();
            return;
        }

        // Get the current elapsed time in the trajectory
        double currentTime = timer.get();
        PathPlannerTrajectoryState goalState = trajectory.sample(currentTime); // Get the desired state at the current
                                                                               // time

        // Command the robot to follow the sampled trajectory state

        SwerveSample setpoint = new SwerveSample(
                0,
                goalState.pose.getX(),
                goalState.pose.getY(),
                goalState.pose.getRotation().getRadians(),
                goalState.fieldSpeeds.vxMetersPerSecond,
                goalState.fieldSpeeds.vyMetersPerSecond,
                goalState.fieldSpeeds.omegaRadiansPerSecond,
                0,
                0,
                0,
                new double[4],
                new double[4]);

        Robot.swerve.driveToSample(setpoint);
    }

    /**
     * Stops the trajectory execution when the command ends.
     *
     * @param interrupted Indicates whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        currentlyThreading = false;
        timer.stop();
        timer.reset();
        Robot.swerve.setIsOTF(false);

    }

    /**
     * Determines if the trajectory is complete.
     * The command finishes if the trajectory time has elapsed and the robot is
     * within an acceptable error margin.
     *
     * @return true if the trajectory is complete and the robot is close enough to
     *         the target.
     */
    @Override
    public boolean isFinished() {
        return !currentlyThreading && (trajectory == null || !pose.equals(endpoint)) ||
                (!Robot.swerve.getIsOTF());
    }

    public double getTime() {
        return timer.get();
    }
}