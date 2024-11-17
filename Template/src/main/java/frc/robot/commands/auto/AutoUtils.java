package frc.robot.commands.auto;

import java.util.function.Function;

// import com.choreo.lib.Choreo;
import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        // default auto
        chooser.choose("Print");


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
        String defautAuto = "Print";
        // interface for choreo
        chooser = new AutoChooser(factory, "Shuffleboard");
        chooser.addAutoRoutine("My Routine", (AutoFactory factory) -> Autos.getMyRoutine(factory));
        chooser.addAutoRoutine("Print", (AutoFactory factory) -> Autos.getPrint(factory));

     
    }

    public static AutoChooser getChooser() {
        return chooser;
    }

}
