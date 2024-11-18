package frc.robot.subsystems.example.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.example.SubsystemIO;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for an example subsystem's simulation
 * 
 * @author Noah Simon
 */
public class SubsystemSim implements SubsystemIO {

    private FlywheelSim simSystem = new FlywheelSim(
            DCMotor.getNEO(1), 6, 0.04);

    private double appliedVolts = 0;
    private double previousVelocity = 0;
    private double velocity = 0;
    private double conversionFactor = 1;

    public SubsystemSim() {
        System.out.println("[Init] Creating SubsytemSim");
    }

    @Override
    public void updateData(SubsystemData data) {
        simSystem.update(SimConstants.loopPeriodSec);

        // set these to your system's data
        previousVelocity = velocity;
        velocity = simSystem.getAngularVelocityRadPerSec() * conversionFactor;
        data.positionUnits += velocity * 0.02;
        data.velocityUnits = velocity;
        data.accelerationUnits = (velocity - previousVelocity) / 0.02;
        data.currentAmps = simSystem.getCurrentDrawAmps();
        data.inputVolts = appliedVolts;
        data.appliedVolts = appliedVolts;

        // sim has no temperature
        data.tempCelcius = 0.0;

    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        simSystem.setInputVoltage(appliedVolts);

    }
}
