package frc.robot.commands.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
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

    /***
     * 
     * @return The command current selected by the auto chooser
     */
    public static Command getSelectedCommand() {
        return AutoUtils.getChooser().selectedCommand();
    }

    /***
     * A do nothing auto
     * 
     * @param factory the AutoFactory from AutoUtils
     * @return Print Command
     */
    public static Command getPrint(AutoFactory factory) {
        return Commands.print("Print Auto!");
    }

    /***
     * An example, single route auto path
     * 
     * @param factory the AutoFactory from AutoUtils
     * @return "My Routine" Command
     */
    public static Command getMyRoutine(AutoFactory factory) {
        // instaniate our auto loop and trajectories
        AutoRoutine routine = factory.newRoutine("my routine");
        AutoTrajectory trajectory = routine.trajectory("trajectoryName");

        // create our trajectory commands, setting odometry and resetting logging when
        // finished
        Command trajectoryCommand = trajectory.cmd();

        // set the command to begin when the loop enables
        routine.active().onTrue(trajectoryCommand);

        // our final, total command
        Command cmd;

        // adding things together
        cmd = Commands.waitSeconds(1).andThen(Commands.print("Print then Trajectory!"));
        cmd = cmd.andThen(routine.cmd());
        return cmd;
    }

    /***
     * An example muli-route auto path.
     * Based on the results of a boolean supplier, a different 2nd half will occur
     * 
     * @param factory the AutoFactory from AutoUtils
     * @return "Split Routine" Command
     */
    public static Command getSplitRoutine(AutoFactory factory) {
        // becomes AutoRoutine
        AutoRoutine routine = factory.newRoutine("split routine");
        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("split1");
        AutoTrajectory trajectory2a = routine.trajectory("split2a");
        AutoTrajectory trajectory2b = routine.trajectory("split2b");

        Command trajectoy1Command = trajectory1.cmd();
        // active
        routine.active().onTrue(trajectoy1Command);

        trajectory1.done().and(() -> true).onTrue(trajectory2a.cmd());
        trajectory1.done().and(() -> false).onTrue(trajectory2b.cmd());
        return Commands.print("split trajectory auto!").andThen(routine.cmd());

    }
    
    /***
     * A routine that drives straight
     * 
     * @param factory the AutoFactory from AutoUtils
     * @return "Straight" Command
     */
    public static Command getStraight(AutoFactory factory) {
        AutoRoutine routine = factory.newRoutine("straight");
        AutoTrajectory trajectory1 = routine.trajectory("straight");

        Command trajectoy1Command = trajectory1.cmd();

        routine.active().onTrue(Commands.waitSeconds(1).andThen(trajectoy1Command));

        return Commands.print("straight").andThen(routine.cmd());

    }
}
