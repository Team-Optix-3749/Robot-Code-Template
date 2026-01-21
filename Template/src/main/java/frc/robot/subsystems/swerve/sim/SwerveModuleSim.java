package frc.robot.subsystems.swerve.sim;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.swerve.SwerveModuleDataAutoLogged;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.utils.MiscUtils;
import frc.robot.config.SwerveConfig.Drivetrain;
import frc.robot.config.SwerveConfig.Motor;

/**
 * Simulation implementation for swerve modules.
 * Inspired by 6328's Swerve Sim.
 */
public class SwerveModuleSim implements SwerveModuleIO {

    LinearSystem<N1, N1, N1> drivePlant = LinearSystemId.createFlywheelSystem(
            DCMotor.getNEO(1),
            Drivetrain.TRANSLATE_MOI,
            Motor.DRIVE_GEARING);
    FlywheelSim driveSim = new FlywheelSim(drivePlant, DCMotor.getNEO(1), 0.0);

    LinearSystem<N1, N1, N1> turnPlant = LinearSystemId.createFlywheelSystem(
            DCMotor.getNEO(1),
            Drivetrain.ROTATE_MOI,
            Motor.TURN_GEARING);
    FlywheelSim turnSim = new FlywheelSim(turnPlant, DCMotor.getNEO(1), 0.0);

    private final SwerveModuleDataAutoLogged data;

    private LinearVelocity prevDriveVelocity = MetersPerSecond.of(0.0);

    public SwerveModuleSim(int index, SwerveModuleDataAutoLogged moduleData) {
        data = moduleData;
        data.index = index;
    }

    @Override
    public void setDriveVoltage(double volts) {
        double apply = MiscUtils.voltageClamp(volts);
        data.driveDesiredVolts = apply;
        data.driveAppliedVolts = apply;
        driveSim.setInputVoltage(apply);
    }

    @Override
    public void setTurnVoltage(double volts) {
        double apply = MiscUtils.voltageClamp(volts);
        data.turnDesiredVolts = apply;
        data.turnAppliedVolts = apply;
        turnSim.setInputVoltage(apply);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {

    }

    @Override
    public void setTurningBrakeMode(boolean enable) {

    }

    @Override
    public void requestDriveVelocity(double setpoint) {
        // double error = setpoint - getDriveVelocityMetersPerSec();
        // double voltage = MathUtil.clamp(error * 10.0, -12, 12);
        // setDriveVoltage(voltage);
    }

    @Override
    public void requestTurnPosition(Rotation2d setpoint) {
        // double error = setpoint.getRadians() - turnPositionRad;
        // error = MathUtil.angleModulus(error);
        // double voltage = MathUtil.clamp(error * 10.0, -12, 12);
        // setTurnVoltage(voltage);
    }

    @Override
    public void syncEncoderPosition(Rotation2d position) {

    }

    private LinearVelocity getDriveVelocity() {
        var radius = Drivetrain.WHEEL_DIAMETER.div(2);
        var velocity = radius.times(driveSim.getAngularVelocityRadPerSec()).in(Meters);
        return MetersPerSecond.of(velocity);
    }

    @Override
    public void updateData() {
        double deltaT = RobotConfig.General.NOMINAL_LOOP_TIME_S;

        driveSim.update(deltaT);
        turnSim.update(deltaT);

        Rotation2d angleDiff = Rotation2d.fromRadians(turnSim.getAngularVelocityRadPerSec() * deltaT);

        LinearVelocity driveVelocity = getDriveVelocity();
        LinearAcceleration driveAccel = driveVelocity.minus(prevDriveVelocity).div(Seconds.of(deltaT));
        prevDriveVelocity = driveVelocity;

        data.drivePosition = data.drivePosition.plus(driveVelocity.times(Seconds.of(deltaT)));
        data.driveVelocity = driveVelocity;
        data.driveAcceleration = driveAccel;
        data.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
        data.driveTempCelcius = 0.0;

        data.turnPosition = data.turnPosition.plus(angleDiff);
        data.absoluteEncoderPosition = data.turnPosition;
        data.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        data.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
        data.turnTempCelcius = 0.0;
    }
}