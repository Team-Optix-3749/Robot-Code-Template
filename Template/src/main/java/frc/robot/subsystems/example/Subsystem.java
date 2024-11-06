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

    private ShuffleData<String> exampleData = new ShuffleData<String>(this.getName(), "example data", "example");
    private SubsystemStates state = SubsystemStates.STOP;

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

    public void setVoltage(double volts) {
        subsystemIO.setVoltage(volts);
    }

    public void stop() {
        subsystemIO.setVoltage(0);
    }

    public void setState(SubsystemStates state) {
        this.state = state;
    }

    public void logOtherData(){
        exampleData.set("new value");
    }

    @Override
    public void periodic(){
        subsystemIO.updateData(data);
        logOtherData();
    }

}
