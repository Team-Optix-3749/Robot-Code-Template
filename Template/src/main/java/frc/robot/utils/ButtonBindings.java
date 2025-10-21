package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.RobotConfig.ControlMode;
import frc.robot.config.RobotConfig.Controller;
import frc.robot.config.RobotConfig.RobotType;
import frc.robot.Robot;
import frc.robot.commands.swerve.SwerveDefaultCommand;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;

public final class ButtonBindings {

    private static final CommandXboxController pilot = new CommandXboxController(Controller.PILOT_PORT);
    private static final CommandXboxController operator = new CommandXboxController(Controller.OPERATOR_PORT);

    public static final Alert controllerAlert = new Alert("No controllers connected!", Alert.AlertType.kError);

    /**
     * If both controllers are plugged in (pi and op)
     */
    public static void twoControllerBindings(CommandXboxController piCtl, CommandXboxController opCtl) {
        // Add any dual-controller bindings here.
        // Example:
        bindOnTrue(piCtl.povUp(), Commands.print("Pilot: povUp pressed"));
        bindOnTrue(opCtl.a(), Commands.print("Operator: A pressed"));

        bindOnTrue(piCtl.start(), Robot.swerve::resetGyro);
        bindOnTrue(piCtl.back(), Robot.swerve::syncEncoderPositions);
    }

    public static void oneControllerBindings(CommandXboxController ctl) {
        // Add any pilot-only bindings here.
    }

    public static void simBindings() {
        twoControllerBindings(pilot, operator);
    }

    private static void bindOnTrue(Trigger trigger, Runnable command, Runnable... onFalse) {
        Command onTrueCmd = Commands.runOnce(command);
        trigger.onTrue(onTrueCmd);

        if (onFalse.length > 0) {
            Command onFalseCmd = Commands.runOnce(onFalse[0]);
            trigger.onFalse(onFalseCmd);
        }

        if (onFalse.length > 1) {
            DriverStation.reportWarning(
                    "bindOnTrue called with multiple onFalse commands; only the first will be used.", false);
        }
    }

    private static void bindOnTrue(Trigger trigger, Command command, Command... onFalse) {
        trigger.onTrue(command);

        if (onFalse.length > 0) {
            trigger.onFalse(onFalse[0]);
        }

        if (onFalse.length > 1) {
            DriverStation.reportWarning(
                    "bindOnTrue called with multiple onFalse commands; only the first will be used.", false);
        }
    }

    private static boolean isPilotConnected() {
        return DriverStation.isJoystickConnected(Controller.PILOT_PORT);
    }

    private static boolean isOperatorConnected() {
        return DriverStation.isJoystickConnected(Controller.OPERATOR_PORT);
    }

    private static ControlMode getControlMode() {
        if (MiscUtils.getRobotType() == RobotType.SIM) {
            return ControlMode.SIM;
        }
        boolean pilot = isPilotConnected();
        boolean op = isOperatorConnected();
        if (pilot && op)
            return ControlMode.BOTH;
        if (pilot)
            return ControlMode.PILOT_ONLY;
        if (op)
            return ControlMode.OPERATOR_ONLY;
        return ControlMode.NONE;
    }

    private static CommandXboxController getActiveController() {
        ControlMode mode = getControlMode();

        switch (mode) {
            case OPERATOR_ONLY:
                return operator;
            default:
                return pilot;
        }
    }

    /**
     * Call the appropriate bindings methods and set default commands.
     */
    public static void apply() {
        setDefaultCommands();

        ControlMode mode = getControlMode();

        switch (mode) {
            case BOTH -> twoControllerBindings(pilot, operator);
            case PILOT_ONLY -> oneControllerBindings(pilot);
            case OPERATOR_ONLY -> oneControllerBindings(operator);
            case SIM -> simBindings();
            case NONE -> {
            }
        }

        controllerAlert.set(mode == ControlMode.NONE);

        Logger.recordOutput("ControlMode", mode);
    }

    private static void setDefaultCommands() {
        ControlMode mode = getControlMode();

        switch (mode) {
            case BOTH, PILOT_ONLY, SIM -> {
                Robot.swerve.setDefaultCommand(
                        new SwerveDefaultCommand(
                                () -> getAxis(pilot, Axis.kLeftX),
                                () -> getAxis(pilot, Axis.kLeftY),
                                () -> getAxis(pilot, Axis.kRightX)));
            }
            default -> {

            }
        }
    }

    private static double getAxis(CommandXboxController ctl, Axis axis) {
        return applyDeadbandInternal(ctl.getRawAxis(axis.value));
    }

    private static double applyDeadbandInternal(double v) {
        return MathUtil.applyDeadband(v, Controller.DEADBAND);
    }

}
