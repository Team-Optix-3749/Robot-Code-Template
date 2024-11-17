package frc.robot.commands.auto;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Bassically AutoRoutineGenerator, I think. I just don't understand how to use
 * that interface and this works
 */
public class Autos {
    public static Command getPrint(AutoFactory factory) {
        return Commands.print("Print Auto!");
    }

    public static Command getMyRoutine(AutoFactory factory) {
        // instaniate our auto loop and trajectories
        AutoLoop loop = factory.newLoop("auto");
        AutoTrajectory trajectory = factory.trajectory("trajectoryName", loop);

        // create our trajectory commands, setting odometry and resetting logging when finished
        Command trajectoryCommand = AutoUtils.addResetLoggingCommand(
                AutoUtils.makeStartingTrajectoryCommand(trajectory));

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
