package frc.robot.commands.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Class containing our auto routines. Referenced by the auto selector and
 * potentially robot container
 * 
 * @author Noah Simon
 */
public class Autos {
    public static Command getPrint(AutoFactory factory) {
        return Commands.print("Print Auto!");
    }

    public static Command getMyRoutine(AutoFactory factory) {
        // instaniate our auto loop and trajectories
        AutoLoop loop = factory.newLoop("auto");
        AutoTrajectory trajectory = factory.trajectory("trajectoryName", loop);
        AutoTrajectory splitPath = factory.trajectory("splitTrajectory", loop);
        trajectory.atPose("Marker").onTrue(AutoUtils.addResetLoggingCommand(splitPath.cmd()));

        // create our trajectory commands, setting odometry and resetting logging when
        // finished
        Command trajectoryCommand = AutoUtils.makeStartingTrajectoryCommand(trajectory);

        // set the command to begin when the loop enables
        loop.enabled().onTrue(trajectoryCommand);

        // our final, total command
        Command cmd;

        // adding things together
        cmd = Commands.print("Print then Trajectory!");
        cmd = cmd.andThen(loop.cmd());
        return cmd;
    }
}
