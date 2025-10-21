package frc.robot.commands.auto;

import java.util.function.Consumer;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.auto.AutoRoutine;
import choreo.trajectory.SwerveSample;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.utils.MiscUtils;

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


    public static AutoChooser getChooser() {
        return chooser;
    }

    public static AutoFactory getAutoFactory() {
        return factory;
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

    public static Command getSingleTrajectory(String trajectoryName) {
        AutoRoutine routine = factory.newRoutine(trajectoryName);
        AutoTrajectory trajectory1 = routine.trajectory(trajectoryName);

        Command trajectoy1Command = trajectory1.cmd();

        routine.active().onTrue(
                factory.resetOdometry(trajectoryName).andThen(
                        trajectoy1Command));

        Logger.recordOutput("Auto/" + trajectoryName + "/InitialPose", trajectory1.getInitialPose().get());
        return Commands.print(trajectoryName).andThen(routine.cmd());

    }
}
