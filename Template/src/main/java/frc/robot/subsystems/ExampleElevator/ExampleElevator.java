package frc.robot.subsystems.ExampleElevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.config.ExampleElevatorConfig.ElevatorControl;
import frc.robot.config.ExampleElevatorConfig.ElevatorSpecs;
import frc.robot.config.ExampleElevatorConfig.ElevatorStates;
import frc.robot.config.ExampleElevatorConfig.ElevatorControl.ControlConfig;
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
    ControlConfig config = ElevatorControl.CONTROL_CONFIG;
    /*
     * lucky for us WPILib has an elevator feedforward controller, most subsystems
     * don'tconfig.config.config.
     */
    ElevatorFeedforward feedforward = new ElevatorFeedforward(config.kS, config.kG, config.kV, config.kA);
    PIDController pid = new PIDController(config.kP, config.kI, config.kD);

    /* last is state and any other variables needed */
    ElevatorStates currentState = ElevatorStates.STOPPED;

    // stuff to run the akit visualization
    public LoggedMechanism2d mech2d = new LoggedMechanism2d(3, 5);
    public LoggedMechanismRoot2d root = mech2d.getRoot("ElevatorRoot", 1.5, 0);
    public LoggedMechanismLigament2d ligament = new LoggedMechanismLigament2d("Elevator",
            ElevatorSpecs.STARTING_HEIGHT_M, 90);
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

    public double getHeightM() {
        return data.position.getY();
    }

    public Translation2d getPosition() {
        return data.position;
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

        double error = Math.abs(pid.getSetpoint() - data.position.getY());
        return error < RobotConfig.ACCURACY.ELEVATOR_TOLERANCE_M &&
                isStopped();
    }

    public boolean isStopped() {
        return MiscUtils.isStopped(data.velocityMPS, RobotConfig.ACCURACY.DEFAULT_MOVEMENT_TOLERANCE_MPS);
    }

    public void moveToGoal() {
        if (currentState == ElevatorStates.STOPPED) {
            io.setVoltage(0);
            return;
        }

        pid.setSetpoint(getHeightM());

        double pidOutput = pid.calculate(data.position.getY());
        double feedforwardOutput = feedforward.calculate(data.velocityMPS);

        double voltageOutput = pidOutput + feedforwardOutput;
        io.setVoltage(voltageOutput);
    }

    public void updateMechanism() {
        ligament.setLength(data.position.getY());

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
