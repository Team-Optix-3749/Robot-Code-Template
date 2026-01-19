package frc.robot.subsystems.ExampleElevator;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.config.ExampleElevatorConfig.ElevatorControl;
import frc.robot.config.ExampleElevatorConfig.ElevatorSpecs;
import frc.robot.config.ExampleElevatorConfig.ElevatorStates;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.RobotType;
import frc.robot.subsystems.ExampleElevator.ElevatorIO.ElevatorData;
import frc.robot.subsystems.ExampleElevator.real.ElevatorReal;
import frc.robot.subsystems.ExampleElevator.sim.ElevatorSiml;
import frc.robot.utils.MiscUtils;

public class ExampleElevator {
    /* subsystem controllers and data go at the top */
    ElevatorIO io;
    ElevatorDataAutoLogged data = new ElevatorDataAutoLogged();

    /* next comes open/closed loop controllers such as pid and ff */
    // only reason for this is because its more readable
    MiscUtils.ControlConfig config = ElevatorControl.CONTROL_CONFIG.get();
    /*
     * lucky for us WPILib has an elevator feedforward controller, most subsystems
     * don'tconfig.config.config.
     */
    ElevatorFeedforward feedforward = new ElevatorFeedforward(config.kS, config.kG, config.kV, config.kA);
    ProfiledPIDController profile = new ProfiledPIDController(config.kP, config.kI, config.kD,
            new Constraints(ElevatorControl.MAX_VELOCITY.in(MetersPerSecond),
                    ElevatorControl.MAX_ACCEL.in(MetersPerSecondPerSecond)));

    /* last is state and any other variables needed */
    ElevatorStates currentState = ElevatorStates.STOPPED;

    // stuff to run the akit visualization
    public LoggedMechanism2d mech2d = new LoggedMechanism2d(3, 5);
    public LoggedMechanismRoot2d root = mech2d.getRoot("ElevatorRoot", 1.5, 0);
    public LoggedMechanismLigament2d ligament = new LoggedMechanismLigament2d("Elevator",
            ElevatorSpecs.STARTING_HEIGHT.in(Meters), 90);
    {
        root.append(ligament);
    }

    /* under all variables is the actual code */
    public ExampleElevator() {
        if (MiscUtils.getRobotType() == RobotType.REAL) {
            io = new ElevatorReal(data);
        } else {
            io = new ElevatorSiml(data);
        }
    }

    /* GETTERS GO AT THE TOP */

    public ElevatorStates getState() {
        return currentState;
    }

    // ElevatorDataAutoLogged extends ElevatorData, so this works
    // We don't want to return the entire AutoLog object because those methods are
    // not meant to be accessed by the user
    public ElevatorData getData() {
        return data;
    }

    public Distance getHeight() {
        return data.height;
    }

    public Translation2d getPosition() {
        return new Translation2d(ElevatorSpecs.MOUNT_OFFSET.getX(), data.height.in(Meters));
    }

    /* SETTERS GO AFTER GETTERS */

    public void setState(ElevatorStates state) {
        // check for valid state
        if (state == null || state == ElevatorStates.STOPPED) {
            throw new IllegalArgumentException("Cannot set elevator state to 'null' or 'STOPPED'");
        }

        currentState = state;
    }

    /* OTHER METHODS GO AFTER SETTERS */

    public void stop() {
        currentState = ElevatorStates.STOPPED;
    }

    public boolean isStableState() {
        // check if within the tolerance of the setpoint and is not moving

        double error = Math.abs(profile.getSetpoint().position - data.height.in(Meters));
        return error < RobotConfig.ACCURACY.ELEVATOR_TOLERANCE.in(Meters) &&
                isStopped();
    }

    public boolean isStopped() {
        return MiscUtils.isStopped(data.velocity.in(MetersPerSecond),
                RobotConfig.ACCURACY.DEFAULT_MOVEMENT_TOLERANCE.in(MetersPerSecond));
    }

    public void moveToGoal() {
        if (currentState == ElevatorStates.STOPPED) {
            io.setVoltage(0);
            return;
        }

        State firstState = profile.getSetpoint();
        double pidOutput = profile.calculate(getHeight().in(Meters));

        State nextState = profile.getSetpoint();
        double feedforwardOutput = feedforward.calculateWithVelocities(firstState.velocity,
                nextState.velocity);

        double voltageOutput = pidOutput + feedforwardOutput;
        io.setVoltage(voltageOutput);
    }

    public void updateMechanism() {
        ligament.setLength(getHeight().in(Meters));

        Logger.recordOutput("Elevator/mechanism", mech2d);
    }

    public void periodic() {
        // always get latest data first
        io.updateData();

        // send logged data to ascope
        Logger.processInputs("Elevator", data);

        // then do control
        moveToGoal();

        // finally update any logging or visualizations
        updateMechanism();
    }
}
