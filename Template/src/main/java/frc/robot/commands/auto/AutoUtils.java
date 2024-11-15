package frc.robot.commands.auto;

// import com.choreo.lib.Choreo;
import choreo.Choreo;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
public class AutoUtils {
    // make sure to properly log the robot's setpoints

    private static AutoFactory factory;

    public static void setupFactory(){
        AutoFactory.AutoBindings bindings = new AutoFactory.AutoBindings();
        bindings.bind("Marker", Commands.print("Marker Passed"));

        factory = new AutoFactory(() -> Robot.swerve.getPose(), null, null, null, null, null);
    }

    public static AutoFactory getFactory(){
        return factory;
    }

}
