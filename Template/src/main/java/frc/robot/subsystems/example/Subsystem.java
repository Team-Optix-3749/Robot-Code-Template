package frc.robot.subsystems.example;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.example.SubsystemConstants.SubsystemStates;
import frc.robot.subsystems.example.SubsystemIO.SubsystemData;
import frc.robot.subsystems.example.real.SubsystemSparkMax;
import frc.robot.subsystems.example.sim.SubsystemSim;
import frc.robot.utils.ShuffleData;

// Example Subsystem
public class Subsystem extends SubsystemBase {

    private SubsystemIO subsystemIO;
    private SubsystemData data = new SubsystemData();
    private SubsystemStates state = SubsystemStates.STOP;

    private ShuffleData<String> exampleDataLog = new ShuffleData<String>(this.getName(), "example data", "example");
    private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state", state.name());

    public Subsystem() {
        if (Robot.isSimulation()) {
            subsystemIO = new SubsystemSim();

        } else {
            subsystemIO = new SubsystemSparkMax();
        }
    }

    public double getPositionRad() {
        return data.positionUnitsLog.get();
    }

    public double getVelocityRadPerSec() {
        return data.velocityUnitsLog.get();
    }

    public SubsystemStates getState() {
        return state;
    }

    // returns true when the state is reached
    public boolean getIsStableState() {
        return true;
    }

    public void setVoltage(double volts) {
        subsystemIO.setVoltage(volts);
    }

    public void stop() {
        subsystemIO.setVoltage(0);
    }

    public void setState(SubsystemStates state) {
        this.state = state;
    }

    public void logOtherData() {
        exampleDataLog.set("new value");
        stateLog.set(state.name());
    }

    public void runStateStop() {
        setVoltage(0);
    }

    public void runStateGo() {
        setVoltage(6);
    }

    @Override
    public void periodic() {
        subsystemIO.updateData(data);

        if (state == SubsystemStates.GO) {
            runStateGo();
        } else if (state == SubsystemStates.STOP) {
            runStateStop();
        }

        logOtherData();
    }

}
