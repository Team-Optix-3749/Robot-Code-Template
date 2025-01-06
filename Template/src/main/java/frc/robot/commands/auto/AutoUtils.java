package frc.robot.commands.auto;

import java.util.function.Consumer;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.auto.AutoRoutine;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    }

    /**
     * 
     * @return the auto selector object
     */
    public static AutoChooser getChooser() {
        return chooser;
    }

    public static AutoFactory getAutoFactory() {
        return factory;
    }

    /**
     * setup the choreo factor object with bindings, controller, etc.
     */
    private static void setupFactory() {
        // what commands run on what markers
        AutoBindings bindings = new AutoFactory.AutoBindings();
        bindings.bind("Marker", Commands.print("Marker Passed"));

        /**
         * Swerve Pose Supplier
         * Reset Odometry Method
         * Swerve Controller (PID and set chassis speeds)
         * Alliance Supplier for swapping
         * Swerve Subsystem for scheduling
         * Bindings, created above
         */

        // will now take a reset odometry

        factory = new AutoFactory(() -> Robot.swerve.getPose(),
                (Pose2d startingPose) -> Robot.swerve.setOdometry(startingPose),
                (SwerveSample sample) -> Robot.swerve.followSample(sample),
                true,
                Robot.swerve,
                bindings);

    }

    /**
     * setup the choreo auto chooser and assign it to the Shuffleboard/Auto tab of
     * networktables
     */
    private static void setupChooser() {
        // interface for choreo

        // Made sendable, use SmartDashbaord now
        chooser = new AutoChooser();
        SmartDashboard.putData("Auto: Auto Chooser", chooser);
        chooser.addCmd("My Routine", () -> Autos.getMyRoutine());
        chooser.addCmd("Print", () -> Autos.getPrint());
        chooser.addCmd("Split", () -> Autos.getSplitRoutine());
        chooser.addCmd("Straight", () -> Autos.getStraight());
        // Default
        chooser.select("Straight");

    }

    public static Command getSingleTrajectory(String trajectoryName) {
        AutoRoutine routine = factory.newRoutine(trajectoryName);
        AutoTrajectory trajectory1 = routine.trajectory(trajectoryName);

        Command trajectoy1Command = trajectory1.cmd();

        routine.active().onTrue(
                factory.resetOdometry(trajectoryName).andThen(
                        trajectoy1Command));
        
        System.out.println(trajectory1.getInitialPose().get());
        return Commands.print(trajectoryName).andThen(routine.cmd());

    }

}
