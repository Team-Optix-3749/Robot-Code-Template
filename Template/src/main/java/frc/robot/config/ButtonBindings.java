package frc.robot.config;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.RobotConfig.ControlMode;
import frc.robot.config.RobotConfig.Input;
import frc.robot.config.RobotConfig.RobotType;
import frc.robot.utils.MiscUtils;
import frc.robot.utils.OptixSpark;
import frc.robot.Robot;
import frc.robot.commands.swerve.OnTheFly;
import frc.robot.commands.swerve.TeleopCommand;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class ButtonBindings {
    private static final Trigger hasDualControllers = new Trigger(() -> DriverStation.getJoystickType(1) > 1);

    private static final CommandXboxController pilot = new CommandXboxController(Input.PILOT_PORT);
    private static final CommandXboxController operator = new CommandXboxController(Input.OPERATOR_PORT);

    public static final Alert controllerAlert = new Alert("No controllers connected!", Alert.AlertType.kError);

    /**
     * If both controllers are plugged in (pi and op)
     */
    public static void twoControllerBindings(CommandXboxController piCtl, CommandXboxController opCtl) {
        // Add any dual-controller bindings here.
        // Example:
        Bind.button(piCtl.povUp()).onTrue(Commands.print("Pilot: povUp pressed"));
        Bind.button(piCtl.a()).onTrue(new OnTheFly(new Pose2d(8.0,
                4.0,
                Rotation2d.fromDegrees(0))));
        Bind.button(opCtl.a()).onTrue(Commands.print("Operator: A pressed"));

        Bind.button(piCtl.start()).onTrue(Robot.swerve::resetGyro);
        Bind.button(piCtl.back()).onTrue(Robot.swerve::syncEncoderPositions);
    }

    public static void oneControllerBindings(CommandXboxController ctl) {
        // Add any pilot-only bindings here.

        Bind.button(ctl.start()).onTrue(Robot.swerve::resetGyro);

        Bind.button(ctl.b()).switchCommands(() -> {
            Robot.intakeMotor.setVoltage(7.5);
        }, () -> {
            Robot.intakeMotor.setVoltage(0);
        });
    }

    public static void simBindings() {
        twoControllerBindings(pilot, operator);
    }

    private static boolean isPilotConnected() {
        return DriverStation.isJoystickConnected(Input.PILOT_PORT);
    }

    private static boolean isOperatorConnected() {
        return DriverStation.isJoystickConnected(Input.OPERATOR_PORT);
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
        hasDualControllers.onChange(Commands.runOnce(ButtonBindings::apply));

        ControlMode mode = getControlMode();
        setDefaultCommands();

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
                        new TeleopCommand(
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
        return MathUtil.applyDeadband(v, Input.DEADBAND);
    }

    public static class Bind {
        private final Trigger trigger;

        boolean switchState = false;

        private Bind(Trigger trigger) {
            this.trigger = trigger;
        }

        public static Bind button(Trigger trigger) {
            return new Bind(trigger);
        }

        public Bind onTrue(Command command) {
            trigger.onTrue(command);
            return this;
        }

        public Bind onTrue(Runnable runnable) {
            return onTrue(Commands.runOnce(runnable));
        }

        public Bind onFalse(Command command) {
            trigger.onFalse(command);
            return this;
        }

        public Bind onFalse(Runnable runnable) {
            return onFalse(Commands.runOnce(runnable));
        }

        public Bind whileHeld(Command command) {
            trigger.whileTrue(command);
            return this;
        }

        public Bind whileHeld(Runnable runnable) {
            return whileHeld(Commands.run(runnable));
        }

        public Bind toggleOnTrue(Command command) {
            trigger.toggleOnTrue(command);
            return this;
        }

        public Bind switchCommands(Command commandA, Command commandB) {
            trigger.onTrue(Commands.either(commandA, commandB, () -> {
                switchState = !switchState;
                return switchState;
            }));
            return this;
        }

        public Bind switchCommands(Runnable commandA, Runnable commandB) {
            switchCommands(
                    Commands.runOnce(commandA),
                    Commands.runOnce(commandB));
            return this;
        }

        // Add more binding types here as needed (e.g., toggleOnRelease, etc.)
    }
}
