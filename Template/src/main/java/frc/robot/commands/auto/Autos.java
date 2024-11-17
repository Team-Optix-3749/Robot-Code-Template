package frc.robot.commands.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import frc.robot.Robot;
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
        AutoLoop loop = factory.newLoop("auto");
        AutoTrajectory trajectory = factory.trajectory("trajectoryName", loop);
        loop.enabled().onTrue(trajectory.cmd());
        Command cmd = Commands.print("Print then Trajectory!");
        cmd = cmd.andThen(loop.cmd());
        return cmd;
    }
}
