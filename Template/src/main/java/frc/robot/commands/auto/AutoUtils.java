package frc.robot.commands.auto;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.config.RobotConfig.ACCURACY;

public class AutoUtils {
    private static AutoFactory factory;
    private static AutoChooser chooser;
    private static final Map<String, Supplier<Command>> eventMarkerCommands = new HashMap<>();

    /**
     * Initializes the auto factory and chooser. Call once during robot init.
     */
    public static void initAutoUtils() {
        setupFactory();
        setupChooser();
    }

    private static void setupFactory() {
        factory = new AutoFactory(
                Robot.swerve::getPose,
                Robot.swerve::setOdometry,
                (SwerveSample sample) -> Robot.swerve.driveToSample(sample),
                true,
                Robot.swerve);
    }

    private static void setupChooser() {
        chooser = new AutoChooser();
        SmartDashboard.putData("Auto Chooser", chooser);

        chooser.addCmd("No Auto", () -> Commands.print("[AutoUtils] No Auto Selected"));
        chooser.select("No Auto");
    }

    /**
     * Registers a global event marker command that can be used across all
     * trajectories.
     */
    public static void registerEventMarker(String name, Supplier<Command> commandSupplier) {
        eventMarkerCommands.put(name, commandSupplier);
    }

    /**
     * Registers a global event marker command that can be used across all
     * trajectories.
     */
    public static void registerEventMarker(String name, Command command) {
        eventMarkerCommands.put(name, () -> command);
    }

    /**
     * Sets up auto to run when autonomous mode is enabled.
     */
    public static void setupAutoTrigger() {
        edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous()
                .whileTrue(chooser.selectedCommandScheduler());
    }

    /**
     * Schedules the currently selected auto command.
     */
    public static void runSelectedCommand() {
        Command autoCmd = chooser.selectedCommand();
        if (autoCmd != null) {
            autoCmd.schedule();
        } else {
            System.out.println("[AutoUtils] No Auto Command Selected");
        }
    }

    /**
     * Registers a routine to the auto chooser.
     */
    public static void registerRoutine(String name, Supplier<AutoRoutine> routineSupplier) {
        chooser.addRoutine(name, routineSupplier);
    }

    /**
     * Registers a command to the auto chooser.
     */
    public static void registerCommand(String name, Supplier<Command> commandSupplier) {
        chooser.addCmd(name, commandSupplier);
    }

    public static AutoChooser getChooser() {
        return chooser;
    }

    public static AutoFactory getFactory() {
        return factory;
    }

    /**
     * Creates a new trajectory builder for easy trajectory configuration.
     */
    public static TrajectoryBuilder trajectory(String name) {
        return new TrajectoryBuilder(name);
    }

    /**
     * Builder class for configuring and creating trajectory commands with event
     * markers.
     */
    public static class TrajectoryBuilder {
        private final String trajectoryName;
        private final Map<String, Supplier<Command>> poseMarkers = new HashMap<>();
        private final Map<Double, Supplier<Command>> timeMarkers = new HashMap<>();
        private boolean resetOdometry = true;
        private boolean holdFinalPose = true;

        private TrajectoryBuilder(String trajectoryName) {
            this.trajectoryName = trajectoryName;
        }

        /**
         * Adds a command to trigger when reaching a named pose marker.
         */
        public TrajectoryBuilder atPose(String poseName, Supplier<Command> command) {
            poseMarkers.put(poseName, command);
            return this;
        }

        /**
         * Adds a command to trigger when reaching a named pose marker.
         */
        public TrajectoryBuilder atPose(String poseName, Command command) {
            return atPose(poseName, () -> command);
        }

        /**
         * Adds a runnable to trigger when reaching a named pose marker.
         */
        public TrajectoryBuilder atPose(String poseName, Runnable command, Subsystem... requirements) {
            return atPose(poseName, Commands.runOnce(command, requirements));
        }

        /**
         * Adds a registered global event marker at a named pose.
         */
        public TrajectoryBuilder atPose(String poseName, String eventMarkerName) {
            if (!eventMarkerCommands.containsKey(eventMarkerName)) {
                System.out.println("[AutoUtils] Warning: Event marker '" + eventMarkerName + "' not registered");
                return this;
            }
            poseMarkers.put(poseName, eventMarkerCommands.get(eventMarkerName));
            return this;
        }

        /**
         * Adds a command to trigger at a specific time in the trajectory.
         */
        public TrajectoryBuilder atTime(double timeSeconds, Supplier<Command> command) {
            timeMarkers.put(timeSeconds, command);
            return this;
        }

        /**
         * Adds a command to trigger at a specific time in the trajectory.
         */
        public TrajectoryBuilder atTime(double timeSeconds, Command command) {
            return atTime(timeSeconds, () -> command);
        }

        /**
         * Adds a runnable to trigger at a specific time in the trajectory.
         */
        public TrajectoryBuilder atTime(double timeSeconds, Runnable command, Subsystem... requirements) {
            return atTime(timeSeconds, Commands.runOnce(command, requirements));
        }

        /**
         * Sets whether to reset odometry at the start of this trajectory.
         */
        public TrajectoryBuilder resetOdometry(boolean reset) {
            this.resetOdometry = reset;
            return this;
        }

        /**
         * Sets whether to hold position at the final pose after trajectory completion.
         */
        public TrajectoryBuilder holdFinalPose(boolean hold) {
            this.holdFinalPose = hold;
            return this;
        }

        /**
         * Builds and returns the trajectory command.
         */
        public Command build() {
            AutoRoutine routine = factory.newRoutine(trajectoryName);
            AutoTrajectory traj = routine.trajectory(trajectoryName);

            for (Map.Entry<String, Supplier<Command>> entry : poseMarkers.entrySet()) {
                traj.atPose(entry.getKey(), ACCURACY.DRIVE_TRANSLATE_TOLERANCE_M, ACCURACY.DRIVE_ROTATION_TOLERANCE_RAD)
                        .onTrue(entry.getValue().get());
            }

            for (Map.Entry<Double, Supplier<Command>> entry : timeMarkers.entrySet()) {
                traj.atTime(entry.getKey()).onTrue(entry.getValue().get());
            }

            Pose2d finalPose = traj.getFinalPose().orElse(Pose2d.kZero);

            if (holdFinalPose) {
                routine.active().onTrue(Commands.sequence(
                        resetOdometry ? traj.resetOdometry() : Commands.none(),
                        traj.cmd(),
                        Commands.run(() -> Robot.swerve.driveToPose(finalPose), Robot.swerve)));
            } else {
                routine.active().onTrue(Commands.sequence(
                        resetOdometry ? traj.resetOdometry() : Commands.none(),
                        traj.cmd()));
            }

            Logger.recordOutput("Auto/" + trajectoryName + "/InitialPose", traj.getInitialPose().orElse(Pose2d.kZero));
            Logger.recordOutput("Auto/" + trajectoryName + "/FinalPose", finalPose);

            return Commands.sequence(
                    Commands.print("[AutoUtils] Running - " + trajectoryName),
                    routine.cmd(),
                    Commands.print("[AutoUtils] Finished - " + trajectoryName));
        }

        /**
         * Builds the command and registers it to the auto chooser.
         */
        public void register() {
            registerCommand(trajectoryName, this::build);
        }
    }
}
