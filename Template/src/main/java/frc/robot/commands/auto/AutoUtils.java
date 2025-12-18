package frc.robot.commands.auto;

import java.util.Map;
import java.util.function.Supplier;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Robot;
import frc.robot.config.RobotConfig.ACCURACY;

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
    // private static LoggedDashboardChooser<Command> loggedChooser = new
    // LoggedDashboardChooser<>("AutoChooser");

    private static Map<String, Command> eventMarkerCommands = Map.of(
            "score", new PrintCommand("Scored"));

    /**
     * run all necessary auto setup methods
     */
    public static void initAutoUtils() {
        setupFactory();
        setupChooser();
        setupAutoTrigger();
    }

    public static void setupFactory() {
        factory = new AutoFactory(
                Robot.swerve::getPose, // A function that returns the current robot pose
                Robot.swerve::setOdometry, // A function that resets the current robot pose to the provided Pose2d
                Robot.swerve::driveToSample, // The drive subsystem trajectory follower
                true, // If alliance flipping should be enabled
                Robot.swerve// The drive subsystem
        );
    }

    public static void setupChooser() {
        chooser = new AutoChooser();
        SmartDashboard.putData("Auto Chooser", chooser);

        // loggedChooser.

        chooser.addCmd("No Auto", () -> Commands.print("[AutoUtils]: No Auto Selected"));
        chooser.addCmd("Sample", () -> getSingleTrajectoryCommand("Sample"));

        chooser.select("No Auto");
    }

    public static void setupAutoTrigger() {
        RobotModeTriggers.autonomous().whileTrue(chooser.selectedCommand());
    }

    /* Adds routine to the auto chooser */
    public static void registerRoutine(String name, Supplier<AutoRoutine> routineSupplier) {
        chooser.addRoutine(name, routineSupplier);
    }

    /* Adds routine to the auto chooser */
    public static void registerCommand(String name, Supplier<Command> commandSupplier) {
        chooser.addCmd(name, commandSupplier);
    }

    public static void applyEventMarkers(AutoTrajectory traj, String... markers) {
        for (String marker : markers) {
            if (!eventMarkerCommands.containsKey(marker)) {
                System.out.println("[AutoUtils]: No command found for event marker: " + marker);
                continue;
            }

            traj.atPose(marker, ACCURACY.DRIVE_TRANSLATE_TOLERANCE_M, ACCURACY.DRIVE_ROTATION_TOLERANCE.getRadians())
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
        return Commands.sequence(
                Commands.print("[AutoUtils]: Running - " + trajectoryName),
                routine.cmd(),
                Commands.print("[AutoUtils]: Finished - " + trajectoryName));
    }
}
