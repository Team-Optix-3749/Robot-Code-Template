package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import frc.robot.subsystems.swerve.SwerveConfig.Control;
import frc.robot.subsystems.swerve.SwerveConfig.Drivetrain;
import frc.robot.subsystems.swerve.SwerveModuleIO.ModuleData;

/**
 * General class for swerve modules that interacts with the
 * interface. Handles all logic relating to individual modules.
 */
public class SwerveModule {
    private final String name;
    private SwerveModuleState desiredState = new SwerveModuleState();

    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0, 0, 0);;
    private final PIDController drivePID = new PIDController(Control.drivePID[0], Control.drivePID[1],
            Control.drivePID[2]);
    private final PIDController turnPID = new PIDController(Control.turnPID[0], Control.turnPID[1],
            Control.turnPID[2]);

    private final SwerveModuleIO moduleIO;
    private ModuleDataAutoLogged moduleData = new ModuleDataAutoLogged();

    /**
     * Constructs a new SwerveModule.
     * 
     * @param index        The module index (0-3)
     * @param SwerveModule The hardware IO implementation
     */
    public SwerveModule(int index, SwerveModuleIO SwerveModule) {
        name = Drivetrain.moduleNames.get(index);
        moduleIO = SwerveModule;

        turnPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * @return The module name (e.g. "Front Left")
     */
    public String getName() {
        return name;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(moduleData.driveVelocityMPerSec, moduleData.turnPosition);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(moduleData.drivePositionM, moduleData.turnPosition);
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    public ModuleData getModuleData() {
        return moduleIO.getData();
    }

    public void setDesiredState(SwerveModuleState state) {
        state.optimize(moduleData.turnPosition);
        setDriveSpeed(desiredState.speedMetersPerSecond);
        setTurnPosition(desiredState.angle);
    }

    /**
     * Sets the drive motor speed with feedforward and PID control.
     * 
     * @param speedMetersPerSecond The target velocity in m/s
     */
    public void setDriveSpeed(double speedMetersPerSecond) {
        double feedforward = driveFF.calculateWithVelocities(moduleData.driveVelocityMPerSec, speedMetersPerSecond);
        double PID = drivePID.calculate(moduleData.driveVelocityMPerSec, speedMetersPerSecond);
        moduleIO.setDriveVoltage(PID + feedforward);
    }

    /**
     * Sets the turn motor position with PID control.
     * 
     * @param positionRad The target angle setpoint
     */
    public void setTurnPosition(Rotation2d positionRad) {
        double PID = turnPID.calculate(moduleData.turnPosition.getRadians(), positionRad.getRadians());
        moduleIO.setTurnVoltage(PID);
    }

    /**
     * Sets the drive motor voltage directly.
     * 
     * @param volts The voltage to apply
     */
    public void setDriveVoltage(double volts) {
        moduleIO.setDriveVoltage(volts);
    }

    /**
     * Sets the turn motor voltage directly.
     * 
     * @param volts The voltage to apply
     */
    public void setTurnVoltage(double volts) {
        moduleIO.setTurnVoltage(volts);
    }

    /**
     * Enables or disables brake mode on both motors.
     * 
     * @param enabled True to enable brake mode, false for coast
     */
    public void setBrakeMode(boolean enabled) {
        moduleIO.setDriveBrakeMode(enabled);
        moduleIO.setTurningBrakeMode(enabled);
    }

    /**
     * Syncs the relative encoder with the absolute encoder.
     */
    public void syncEncoderPosition() {
        moduleIO.syncEncoderPosition();
    }

    /**
     * Stops both drive and turn motors.
     */
    public void stop() {
        setDriveVoltage(0);
        setTurnVoltage(0);
    }

    /**
     * Updates module data from hardware.
     * Called periodically by the swerve subsystem.
     */
    public void periodic() {
        moduleIO.updateData(moduleData);
        Logger.processInputs("Swerve/Module " + moduleData.index, moduleData);
    }
}