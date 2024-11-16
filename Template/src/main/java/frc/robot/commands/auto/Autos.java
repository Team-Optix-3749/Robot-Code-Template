package frc.robot.commands.auto;

import java.util.function.Function;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import choreo.auto.AutoChooser.AutoRoutineGenerator;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import choreo.Choreo;

/**
 * Bassically AutoRoutineGenerator, I think. I just don't understand how to use
 * that interface and this works
 */
public class Autos {
    public static Command getPrint(AutoFactory factory) {
        return Commands.print("Auto!");
    }

    public static Command getMyRoutine(AutoFactory factory) {
        AutoLoop loop = factory.newLoop("auto");
        AutoTrajectory trajectory = factory.trajectory("auto", loop);
        loop.enabled().onTrue(trajectory.cmd());
        return Commands.print("Print then Trajectory!").andThen(loop.cmd());
    }
}
