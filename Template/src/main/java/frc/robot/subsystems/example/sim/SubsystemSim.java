package frc.robot.subsystems.example.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.subsystems.example.SubsystemIO;
import frc.robot.utils.MiscConstants.SimConstants;

public class SubsystemSim implements SubsystemIO {

    private LinearSystemSim simSystem = new LinearSystemSim<>(null);

    private double appliedVolts = 0;
    private double previousVelocity = 0;

    public SubsystemSim() {
        System.out.println("[Init] Creating SubsytemSim");
    }

    @Override
    public void updateData(SubsystemData data) {
        simSystem.update(SimConstants.loopPeriodSec);

        // set these to your system's data
        data.positionUnitsLog.set(0.0);
        data.velocityUnitsLog.set(0.0);
        data.currentAmpsLog.set(simSystem.getCurrentDrawAmps());

        data.inputVoltsLog.set(appliedVolts);
        data.appliedVoltsLog.set(appliedVolts);

        // sim has no temperature
        data.tempCelciusLog.set(0.0);

    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        // simSystem.setInputVoltage(appliedVolts)

    }
}
