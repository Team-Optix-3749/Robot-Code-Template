package frc.robot.subsystems.example.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.example.ExampleSubsystemIO;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for an example subsystem's simulation
 * 
 * @author Noah Simon
 */
public class SubsystemSim implements ExampleSubsystemIO {

    LinearSystem<N1, N1, N1> simPlant = LinearSystemId.createFlywheelSystem(
            DCMotor.getNEO(1), // Motor
            0.04, // J (moment of inertia)
            6 // Gear ratio
    );
    FlywheelSim sim = new FlywheelSim(
            simPlant, // The linear system
            DCMotor.getNEO(1), // The motor (gearbox) model
            0.0 // Optional noise in sensor measurements
    );
    private double appliedVolts = 0;
    private double previousVelocity = 0;
    private double velocity = 0;
    private double conversionFactor = 1;

    public SubsystemSim() {
        System.out.println("[Init] Creating SubsytemSim");
    }

    @Override
    public void updateData(SubsystemData data) {
        sim.update(SimConstants.loopPeriodSec);

        // set these to your system's data
        previousVelocity = velocity;
        velocity = sim.getAngularVelocityRadPerSec() * conversionFactor;
        data.positionUnits += velocity * 0.02;
        data.velocityUnits = velocity;
        data.accelerationUnits = (velocity - previousVelocity) / 0.02;
        data.currentAmps = sim.getCurrentDrawAmps();
        data.inputVolts = appliedVolts;
        data.appliedVolts = appliedVolts;

        // sim has no temperature
        data.tempCelcius = 0.0;

    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInputVoltage(appliedVolts);

    }
}
