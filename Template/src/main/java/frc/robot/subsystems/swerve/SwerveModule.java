package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import frc.robot.config.SwerveConfig.Control;
import frc.robot.config.SwerveConfig.Drivetrain;
import frc.robot.subsystems.swerve.real.SwerveModuleSpark;
import frc.robot.subsystems.swerve.sim.SwerveModuleSim;
import frc.robot.utils.MiscUtils;

/**
 * General class for swerve modules that interacts with the
 * interface. Handles all logic relating to individual modules.
 */
public class SwerveModule {
    private final String name;
    private SwerveModuleState desiredState = new SwerveModuleState();

    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(
            Control.MODULE_DRIVE_KS,
            Control.MODULE_DRIVE_KV,
            Control.MODULE_DRIVE_KA);
    private final PIDController drivePID = new PIDController(Control.MODULE_DRIVE_PID[0], Control.MODULE_DRIVE_PID[1],
            Control.MODULE_DRIVE_PID[2]);
    private final PIDController turnPID = new PIDController(Control.MODULE_TURN_PID[0], Control.MODULE_TURN_PID[1],
            Control.MODULE_TURN_PID[2]);

    private final SwerveModuleIO moduleIO;
    private final SwerveModuleDataAutoLogged moduleData = new SwerveModuleDataAutoLogged();

    private double index;

    LoggedNetworkNumber drive_ks_value;
    LoggedNetworkNumber drive_kv_value;
    LoggedNetworkNumber drive_ka_value;
    
    LoggedNetworkNumber drive_p_value;
    LoggedNetworkNumber drive_i_value;
    LoggedNetworkNumber drive_d_value;

    LoggedNetworkNumber turn_p_value;
    LoggedNetworkNumber turn_i_value;
    LoggedNetworkNumber turn_d_value;

    /**
     * Constructs a new SwerveModule.
     * 
     * @param index        The module index (0-3)
     * @param SwerveModule The hardware IO implementation
     */
    public SwerveModule(int index, SwerveModuleType type) {
        name = Drivetrain.MODULE_NAMES.get(index);
        this.index = index;

        switch (type) {
            case SIM:
                moduleIO = new SwerveModuleSim(index, moduleData);
                break;
            // case FLEX:
            // moduleIO = new SwerveModuleFlex(index, moduleData);
            // break;
            case SPARK:
            default:
                moduleIO = new SwerveModuleSpark(index, moduleData);
                break;
        }

        Logger.recordMetadata("Swerve/Module " + name + "/Implementation", type.name());

        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        drive_ks_value = new LoggedNetworkNumber(
                "Swerve/Module #" + index + "/KS",
                Control.MODULE_DRIVE_KS);
        drive_kv_value = new LoggedNetworkNumber(
                "Swerve/Module #" + index + "/kV",
                Control.MODULE_DRIVE_KV);
        drive_ka_value = new LoggedNetworkNumber(
                "Swerve/Module #" + index + "/KA",
                Control.MODULE_DRIVE_KA);

        drive_p_value = new LoggedNetworkNumber(
                "Swerve/Module Drive #" + index + "/P",
                Control.MODULE_DRIVE_PID[0]);
        drive_i_value = new LoggedNetworkNumber(
                "Swerve/Module Drive #" + index + "/I",
                Control.MODULE_DRIVE_PID[1]);
        drive_d_value = new LoggedNetworkNumber(
                "Swerve/Module Drive #" + index + "/D",
                Control.MODULE_DRIVE_PID[2]);

        turn_p_value = new LoggedNetworkNumber(
                "Swerve/Module Turn #" + index + "/P",
                Control.MODULE_TURN_PID[0]);
        turn_i_value = new LoggedNetworkNumber(
                "Swerve/Module Turn #" + index + "/I",
                Control.MODULE_TURN_PID[1]);
        turn_d_value = new LoggedNetworkNumber(
                "Swerve/Module Turn #" + index + "/D",
                Control.MODULE_TURN_PID[2]);
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

    public SwerveModuleDataAutoLogged getModuleData() {
        return moduleData;
    }

    public void setDesiredState(SwerveModuleState state) {
        desiredState = state;
        desiredState.optimize(moduleData.turnPosition);
    }

    /**
     * Sets the drive motor speed with feedforward and PID control.
     * 
     * @param speedMetersPerSecond The target velocity in m/s
     */
    public void setDriveSpeed(double speedMetersPerSecond) {
        if (MiscUtils.isStopped(speedMetersPerSecond)) {
            moduleIO.setDriveVoltage(0.0);
            return;
        }

        double feedforward = driveFF.calculateWithVelocities(moduleData.driveVelocityMPerSec, speedMetersPerSecond);
        double PID = drivePID.calculate(moduleData.driveVelocityMPerSec, speedMetersPerSecond);
        moduleIO.setDriveVoltage(PID + feedforward);
        Logger.recordOutput("Swerve/Module " + name + "/Drive FF Volts", feedforward);
        Logger.recordOutput("Swerve/Module " + name + "/Drive PID Volts", PID);
        Logger.recordOutput("Swerve/Module " + name + "/Volts", PID + feedforward);
    }

    /**
     * Sets the turn motor position with PID control.
     * 
     * @param positionRad The target angle setpoint
     */
    public void setTurnPosition(Rotation2d positionRad) {
        double PID = turnPID.calculate(moduleData.turnPosition.getRadians(), positionRad.getRadians());
        moduleIO.setTurnVoltage(PID);
        Logger.recordOutput("Swerve/Module " + name + "/Turn Volts", PID);
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
     * Update inputs.
     */
    public void updateInputs() {
        moduleIO.updateData();
        Logger.processInputs("Swerve/Module " + name, moduleData);
    }

    /**
     * Updates module data from hardware.
     * Called periodically by the swerve subsystem.
     */
    public void periodic() {
        drivePID.setPID(drive_p_value.get(), drive_i_value.get(), drive_d_value.get());
        turnPID.setPID(turn_p_value.get(), turn_i_value.get(), turn_d_value.get());

        driveFF.setKs(drive_ks_value.get());
        driveFF.setKa(drive_ka_value.get());
        driveFF.setKv(drive_kv_value.get());

        setDriveSpeed(desiredState.speedMetersPerSecond);
        setTurnPosition(desiredState.angle);
        updateInputs();
    }
}