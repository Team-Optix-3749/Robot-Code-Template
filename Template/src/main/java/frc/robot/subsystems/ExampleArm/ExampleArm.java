package frc.robot.subsystems.ExampleArm;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.config.ExampleArmConfig.ArmControl.ControlConfig;
import frc.robot.config.ExampleArmConfig.ArmStates;
import frc.robot.config.ExampleArmConfig;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.RobotType;
import frc.robot.subsystems.ExampleArm.ArmIO.ArmData;
import frc.robot.subsystems.ExampleArm.real.ArmReal;
import frc.robot.subsystems.ExampleArm.sim.ArmSim;
import frc.robot.utils.MiscUtils;

public class ExampleArm {
    /* subsystem controllers and data go at the top */
    ArmIO io;
    ArmDataAutoLogged data = new ArmDataAutoLogged();

    /* next comes open/closed loop controllers such as pid and ff */
    // only reason for this is because its more readable
    ControlConfig config = ExampleArmConfig.ArmControl.CONTROL_CONFIG;

    ArmFeedforward feedforward = new ArmFeedforward(config.kS, config.kG, config.kV, config.kA);
    PIDController pid = new PIDController(config.kP, config.kI, config.kD);

    /* last is state and any other variables needed */
    ArmStates currentState = ArmStates.STOPPED;

    // stuff to run the akit visualization
    public LoggedMechanism2d mech2d = new LoggedMechanism2d(3, 5);
    public LoggedMechanismRoot2d root = mech2d.getRoot("ArmRoot", 1.5, 0);
    public LoggedMechanismLigament2d ligament = new LoggedMechanismLigament2d("Arm",
            10, data.angle.getDegrees());
    {
        root.append(ligament);
    }

    /* under all variables is the actual code */
    public ExampleArm() {
        if (MiscUtils.getRobotType() == RobotType.REAL) {
            io = new ArmReal(data);
        } else {
            io = new ArmSim(data);
        }

        pid.enableContinuousInput(-Math.PI, Math.PI);
    }

    /* GETTERS GO AT THE TOP */

    public ArmStates getState() {
        return currentState;
    }

    public ArmData getData() {
        return data;
    }

    public Rotation2d getAngle() {
        return data.angle;
    }

    public Translation2d getTranslation() {
        double lengthM = 1.0;
        return new Translation2d(lengthM, data.angle);
    }

    /* SETTERS GO AFTER GETTERS */

    public void setState(ArmStates state) {
        // check for valid state
        if (state == null || state == ArmStates.STOPPED) {
            throw new IllegalArgumentException("Cannot set elevator state to 'null' or 'STOPPED'");
        }

        currentState = state;
    }

    /* OTHER METHODS GO AFTER SETTERS */

    public void stop() {
        currentState = ArmStates.STOPPED;
    }

    public boolean isStableState() {
        // check if within the tolerance of the setpoint and is not moving

        double error = Math.abs(pid.getSetpoint() - data.angle.getRadians());
        return error < RobotConfig.ACCURACY.ELEVATOR_TOLERANCE_M &&
                isStopped();
    }

    public boolean isStopped() {
        return MiscUtils.isStopped(data.angularVelocityRadPS, RobotConfig.ACCURACY.DEFAULT_MOVEMENT_TOLERANCE_MPS);
    }

    public void moveToGoal() {
        if (currentState == ArmStates.STOPPED) {
            io.setVoltage(0);
            return;
        }

        pid.setSetpoint(currentState.angle.getRadians());

        double pidOutput = pid.calculate(data.angle.getRadians());
        double feedforwardOutput = feedforward.calculate(data.angle.getRadians(), data.angularVelocityRadPS);

        double voltageOutput = pidOutput + feedforwardOutput;
        io.setVoltage(voltageOutput);
    }

    public void updateMechanism() {
        ligament.setAngle(data.angle);

        Logger.recordOutput("Arm/mechanism", mech2d);
    }

    public void periodic() {
        // always get latest data first
        io.updateData();

        // send logged data to ascope
        Logger.processInputs("Arm", data);

        // then do control
        moveToGoal();

        // finally update any logging or visualizations
        updateMechanism();
    }
}
