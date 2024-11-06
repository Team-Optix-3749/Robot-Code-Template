package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;

/**
 * Util class for button bindings
 *
 * @author Rohin Sood
 */
public class JoystickIO {

    private static final CommandXboxController pilot = new CommandXboxController(0);
    private static final CommandXboxController operator = new CommandXboxController(1);

    public JoystickIO() {
    }

    /**
     * Calls binding methods according to the joysticks connected
     */
    public static void getButtonBindings() {

        if (DriverStation.isJoystickConnected(1)) {
            // if both xbox controllers are connected
            pilotAndOperatorBindings();
        } else if (DriverStation.isJoystickConnected(0)) {
            // if only one xbox controller is connected
            pilotBindings();
        } else if (Robot.isSimulation()) {
            // will show not connected if on sim
            simBindings();
        } else {

        }

        setDefaultCommands();
    }

    /**
     * If both controllers are plugged in (pi and op)
     */
    public static void pilotAndOperatorBindings() {

        // gyro
        pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));

    }

    public static void pilotBindings() {
        pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));
        pilot.startWhileHeld(null);
    }

    public static void simBindings() {

    }

    /**
     * Sets the default commands
     */
    public static void setDefaultCommands() {

        Robot.swerve.setDefaultCommand(
                new SwerveTeleop(
                        () -> -pilot.getLeftX(),
                        () -> -pilot.getLeftY(),
                        () -> -pilot.getRightX()));
    }

}
