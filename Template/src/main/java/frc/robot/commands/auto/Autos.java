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
        AutoLoop loop = factory.newLoop("my routine");
        AutoTrajectory trajectory = factory.trajectory("trajectoryName", loop);

        // create our trajectory commands, setting odometry and resetting logging when
        // finished
        Command trajectoryCommand = AutoUtils.makeStartingTrajectoryCommand(trajectory)
                .andThen(AutoUtils.getResetLoggingCommand());

        // set the command to begin when the loop enables
        loop.enabled().onTrue(trajectoryCommand);

        // our final, total command
        Command cmd;

        // adding things together
        cmd = Commands.waitSeconds(1).andThen(Commands.print("Print then Trajectory!"));
        cmd = cmd.andThen(loop.cmd());
        return cmd;
    }

    public static Command getSplitRoutine(AutoFactory factory) {
        AutoLoop loop = factory.newLoop("split routine");
        AutoTrajectory trajectory1 = factory.trajectory("split1", loop);
        AutoTrajectory trajectory2a = factory.trajectory("split2a", loop);
        AutoTrajectory trajectory2b = factory.trajectory("split2b", loop);

        Command trajectoy1Command = AutoUtils.makeStartingTrajectoryCommand(trajectory1);
        loop.enabled().onTrue(trajectoy1Command);

        trajectory1.done().and(() -> true).onTrue(trajectory2a.cmd());
        trajectory1.done().and(() -> false).onTrue(trajectory2b.cmd());
        return Commands.print("split trajectory auto!").andThen(loop.cmd());

    }

    public static Command getStraight(AutoFactory factory) {
        AutoLoop loop = factory.newLoop("straight");
        AutoTrajectory trajectory1 = factory.trajectory("straight", loop);

        Command trajectoy1Command = AutoUtils.makeStartingTrajectoryCommand(trajectory1);
        
        loop.enabled().onTrue(Commands.waitSeconds(1).andThen(trajectoy1Command));

        return Commands.print("straight").andThen(loop.cmd());

    }
}
