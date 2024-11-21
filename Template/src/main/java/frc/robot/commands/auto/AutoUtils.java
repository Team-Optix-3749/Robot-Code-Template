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
 * All setup and helper methods for auto routines, including the
 * choreo factory and auto selector
 * 
 * @author Noah Simon
 */
public class AutoUtils {
    // make sure to properly log the robot's setpoints

    private static AutoFactory factory;
    private static AutoChooser chooser;

    /**
     * run all necessary auto setup methods
     */
    public static void initAuto() {
        setupFactory();
        setupChooser();
        // default auto choice
        chooser.choose("Straight");

    }

    /**
     * 
     * @return the auto selector object
     */
    public static AutoChooser getChooser() {
        return chooser;
    }

    /**
     * @param trajectory the trajectory that should be made into a command
     * @return that command, but with a setOdometry preface assigned to the
     *         trajectory's initial pose
     */
    public static Command makeStartingTrajectoryCommand(AutoTrajectory trajectory) {
        return Commands.runOnce(
                () -> Robot.swerve.setOdometry(trajectory.getInitialPose().orElseGet(() -> Robot.swerve.getPose())))
                .andThen(trajectory.cmd());
    }

    /**
     * @param cmd the command to add the reset to the end of
     * @return the command, ending with setting all setpoint logs to 0 or far
     *         negatives
     */
    public static Command getResetLoggingCommand() {
        return Commands.runOnce(() -> Robot.swerve.logSetpoints(
                new SwerveSample(-100, 0, -100, 0, 0, 0, 0, 0,
                        0, 0, new double[] { 0.0, 0.0, 0.0, 0.0 }, new double[] { 0.0, 0.0, 0.0, 0.0 })));
    }

    /**
     * setup the choreo factor object with bindings, controller, etc.
     */
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

    /**
     * setup the choreo auto chooser and assign it to the Shuffleboard/Auto tab of
     * networktables
     */
    private static void setupChooser() {
        // interface for choreo
        chooser = new AutoChooser(factory, "Shuffleboard/Auto");
        chooser.addAutoRoutine("My Routine", (AutoFactory factory) -> Autos.getMyRoutine(factory));
        chooser.addAutoRoutine("Print", (AutoFactory factory) -> Autos.getPrint(factory));
        chooser.addAutoRoutine("Split", (AutoFactory factory) -> Autos.getSplitRoutine(factory));
        chooser.addAutoRoutine("Straight", (AutoFactory factory) -> Autos.getStraight(factory));

    }

}
