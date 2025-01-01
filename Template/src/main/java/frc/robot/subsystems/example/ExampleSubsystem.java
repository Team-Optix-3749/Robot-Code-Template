package frc.robot.subsystems.example;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.example.ExampleSubsystemConstants.SubsystemStates;
import frc.robot.subsystems.example.ExampleSubsystemIO.SubsystemData;
import frc.robot.subsystems.example.real.SubsystemSparkMax;
import frc.robot.subsystems.example.sim.SubsystemSim;
import frc.robot.utils.ShuffleData;

/**
 * An example subsystem to model
 * 
 * @author Noah Simon
 */
public class ExampleSubsystem extends SubsystemBase {

    private ExampleSubsystemIO subsystemIO;
    private SubsystemData data = new SubsystemData();
    private SubsystemStates state = SubsystemStates.STOP;

    private ShuffleData<String> currentCommandLog = new ShuffleData<String>(this.getName(), "current command", "None");
    public ShuffleData<Double> positionUnitsLog = new ShuffleData<Double>("Subsystem", "position units", 0.0);
    public ShuffleData<Double> velocityUnitsLog = new ShuffleData<Double>("Subsystem", "velocity units", 0.0);
    public ShuffleData<Double> accelerationUnitsLog = new ShuffleData<Double>("Subsystem", "acceleration units", 0.0);
    public ShuffleData<Double> inputVoltsLog = new ShuffleData<Double>("Subsystem", "input volts", 0.0);
    public ShuffleData<Double> appliedVoltsLog = new ShuffleData<Double>("Subsystem", "applied volts", 0.0);
    public ShuffleData<Double> currentAmpsLog = new ShuffleData<Double>("Subsystem", "current amps", 0.0);
    public ShuffleData<Double> tempCelciusLog = new ShuffleData<Double>("Subsystem", "temp celcius", 0.0);

    private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state", state.name());

    public ExampleSubsystem() {
        if (Robot.isSimulation()) {
            subsystemIO = new SubsystemSim();

        } else {
            subsystemIO = new SubsystemSparkMax();
        }
    }

    public double getPositionRad() {
        return data.positionUnits;
    }

    public double getVelocityRadPerSec() {
        return data.velocityUnits;
    }

    public SubsystemStates getState() {
        return state;
    }

    // returns true when the state is reached
    public boolean getIsStableState() {

        switch (state) {
            case STOP:
                return data.velocityUnits == 0;
            case GO:
                return data.appliedVolts == 6;
            default:
                return false;
        }
    }

    public void setState(SubsystemStates state) {
        this.state = state;
    }

    private void setVoltage(double volts) {
        subsystemIO.setVoltage(volts);
    }

    private void logData() {
        currentCommandLog.set(this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
        positionUnitsLog.set(data.positionUnits);
        velocityUnitsLog.set(data.velocityUnits);
        inputVoltsLog.set(data.inputVolts);
        appliedVoltsLog.set(data.appliedVolts);
        currentAmpsLog.set(data.currentAmps);
        tempCelciusLog.set(data.tempCelcius);

        stateLog.set(state.name());

    }

    private void runState() {
        switch (state) {
            case STOP:
                runStateStop();
                break;
            case GO:
                runStateGo();
                break;
        }
    }

    private void runStateStop() {
        setVoltage(0);
    }

    private void runStateGo() {
        setVoltage(6);
    }

    @Override
    public void periodic() {
        subsystemIO.updateData(data);

        runState();

        logData();
    }

}
