package frc.robot.commands.auto;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.utils.UtilityFunctions;

/**
 * @author Noah Simon
 * @description
 *              All setup and helper methods for auto routines, including the
 *              choreo
 *              factory
 *              and auto selector
 */
public class AutoUtils {
    // make sure to properly log the robot's setpoints

    private static AutoFactory factory;
    private static AutoChooser chooser;

    public static void initAuto() {
        setupFactory();
        setupChooser();
        // default auto choice
        chooser.choose("My Routine");

    }

    public static AutoChooser getChooser() {
        return chooser;
    }

    public static Command makeStartingTrajectoryCommand(AutoTrajectory trajectory) {
        return Commands.runOnce(
                () -> Robot.swerve.setOdometry(trajectory.getInitialPose().orElseGet(() -> Robot.swerve.getPose())))
                .andThen(trajectory.cmd());
    }

    public static Command addResetLoggingCommand(Command cmd) {
        return cmd.andThen(
                Commands.runOnce(() -> Robot.swerve.logSetpoints(
                        new SwerveSample(-100, 0, -100, 0, 0, 0, 0, 0,
                                0, 0, new double[] { 0.0, 0.0, 0.0, 0.0 }, new double[] { 0.0, 0.0, 0.0, 0.0 }))));
    }

    private static void setupFactory() {
        // what commands run on what markers
        AutoBindings bindings = new AutoFactory.AutoBindings();
        bindings.bind("Marker", Commands.print("Marker Passed"));

        /**
         * Swerve Subsystem for scheduling
         * Swerve Pose Supplier
         * Swerve Controller (PID and set chassis speeds)
         * Alliance Supplier for swapping
         * Bindings, created above
         */
        factory = Choreo.createAutoFactory(Robot.swerve, () -> Robot.swerve.getPose(),
                (Pose2d curPose, SwerveSample sample) -> AutoController.choreoController(curPose, sample),
                () -> UtilityFunctions.isRedAlliance(), bindings);

    }

    private static void setupChooser() {
        // interface for choreo
        chooser = new AutoChooser(factory, "Shuffleboard/Auto");
        chooser.addAutoRoutine("My Routine", (AutoFactory factory) -> Autos.getMyRoutine(factory));
        chooser.addAutoRoutine("Print", (AutoFactory factory) -> Autos.getPrint(factory));

    }

}
