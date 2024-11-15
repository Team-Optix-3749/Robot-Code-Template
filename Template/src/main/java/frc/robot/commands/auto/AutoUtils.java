package frc.robot.commands.auto;

// import com.choreo.lib.Choreo;
import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoChooser.AutoRoutineGenerator;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.utils.UtilityFunctions;

public class AutoUtils {
    // make sure to properly log the robot's setpoints

    private static AutoFactory factory;
    private static AutoChooser chooser;

    public static void initAuto() {
        setupFactory();
        setupChooser();
    }

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

    private static void setupChooser() {
        chooser = new AutoChooser(factory, "");
        chooser.addAutoRoutine("My Routine", null);
        chooser.choose("My Routine");

    }

    public static AutoFactory getFactory() {
        return factory;
    }

    public static AutoChooser getChooser() {
        return chooser;
    }

}
