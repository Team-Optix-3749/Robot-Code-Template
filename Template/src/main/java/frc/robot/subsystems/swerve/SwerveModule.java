package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Measure.*;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.config.SwerveConfig.Control;
import frc.robot.config.SwerveConfig.Drivetrain;
import frc.robot.config.RobotConfig;
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

    private MiscUtils.ControlConfig driveConfig = Control.DRIVE_CONFIG.get();
    private MiscUtils.ControlConfig turnConfig = Control.TURN_CONFIG.get();

    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(
            driveConfig.kS,
            driveConfig.kV,
            driveConfig.kA);
    private final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(
            driveConfig.kS,
            driveConfig.kV,
            driveConfig.kA);
    private final PIDController drivePID = new PIDController(driveConfig.kP, driveConfig.kI,
            driveConfig.kD);
    private final PIDController turnPID = new PIDController(turnConfig.kP, turnConfig.kI,
            turnConfig.kD);

    private final SwerveModuleIO moduleIO;
    private final SwerveModuleDataAutoLogged moduleData = new SwerveModuleDataAutoLogged();

    LoggedNetworkNumber tunableDriveKS;
    LoggedNetworkNumber tunableDriveKV;
    LoggedNetworkNumber tunableDriveKA;

    LoggedNetworkNumber tunableDriveKP;
    LoggedNetworkNumber tunableDriveKI;
    LoggedNetworkNumber tunableDriveKD;

    LoggedNetworkNumber tunableTurnKS;
    LoggedNetworkNumber tunableTurnKV;
    LoggedNetworkNumber tunableTurnKA;

    LoggedNetworkNumber tunableTurnKP;
    LoggedNetworkNumber tunableTurnKI;
    LoggedNetworkNumber tunableTurnKD;

    /**
     * Constructs a new SwerveModule.
     * 
     * @param index        The module index (0-3)
     * @param SwerveModule The hardware IO implementation
     */
    public SwerveModule(int index, SwerveModuleType type) {
        name = Drivetrain.MODULE_NAMES.get(index);

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

        Logger.recordMetadata("Swerve/Module " + name + "/Type", type.name());

        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        tunableDriveKS = new LoggedNetworkNumber("Swerve/Tuning/Module " + name + "/Drive/kS", driveConfig.kS);
        tunableDriveKV = new LoggedNetworkNumber("Swerve/Tuning/Module " + name + "/Drive/kV", driveConfig.kV);
        tunableDriveKA = new LoggedNetworkNumber("Swerve/Tuning/Module " + name + "/Drive/kA", driveConfig.kA);
        tunableDriveKP = new LoggedNetworkNumber("Swerve/Tuning/Module " + name + "/Drive/kP", driveConfig.kP);
        tunableDriveKI = new LoggedNetworkNumber("Swerve/Tuning/Module " + name + "/Drive/kI", driveConfig.kI);
        tunableDriveKD = new LoggedNetworkNumber("Swerve/Tuning/Module " + name + "/Drive/kD", driveConfig.kD);

        tunableTurnKS = new LoggedNetworkNumber("Swerve/Tuning/Module " + name + "/Turn/kS", turnConfig.kS);
        tunableTurnKV = new LoggedNetworkNumber("Swerve/Tuning/Module " + name + "/Turn/kV", turnConfig.kV);
        tunableTurnKA = new LoggedNetworkNumber("Swerve/Tuning/Module " + name + "/Turn/kA", turnConfig.kA);
        tunableTurnKP = new LoggedNetworkNumber("Swerve/Tuning/Module " + name + "/Turn/kP", turnConfig.kP);
        tunableTurnKI = new LoggedNetworkNumber("Swerve/Tuning/Module " + name + "/Turn/kI", turnConfig.kI);
        tunableTurnKD = new LoggedNetworkNumber("Swerve/Tuning/Module " + name + "/Turn/kD", turnConfig.kD);
    }

    /**
     * @return The module name (e.g. "Front Left")
     */
    public String getName() {
        return name;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(moduleData.driveVelocity, moduleData.turnPosition);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(moduleData.drivePosition, moduleData.turnPosition);
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
     * @param speed The target velocity in m/s
     */
    public void setDriveSpeed(LinearVelocity speed) {
        if (MiscUtils.isStopped(speed)) {
            moduleIO.setDriveVoltage(0.0);
            return;
        }

        double feedforward = driveFF.calculateWithVelocities(moduleData.driveVelocity.in(MetersPerSecond),
                speed.in(MetersPerSecond));
        double PID = drivePID.calculate(moduleData.driveVelocity.in(MetersPerSecond), speed.in(MetersPerSecond));
        moduleIO.setDriveVoltage(PID + feedforward);
        Logger.recordOutput("Swerve/Module " + name + "/Drive FF Volts", feedforward);
        Logger.recordOutput("Swerve/Module " + name + "/Drive PID Volts", PID);
        Logger.recordOutput("Swerve/Module " + name + "/Drive Desired Volts", PID + feedforward);
    }

    /**
     * Sets the turn motor position with PID control.
     * 
     * @param position The target angle setpoint
     */
    public void setTurnPosition(Rotation2d position) {
        if (MiscUtils.withinMargin(RobotConfig.Accuracy.DRIVE_ROTATION_TOLERANCE.in(Radians), position.getRadians(),
                moduleData.turnPosition.getRadians())) {
            moduleIO.setTurnVoltage(0.0);
            Logger.recordOutput("Swerve/Module " + name + "/Turn Desired Volts", 0);
            return;
        }

        double PID = turnPID.calculate(moduleData.turnPosition.getRadians(), position.getRadians());
        moduleIO.setTurnVoltage(PID);
        Logger.recordOutput("Swerve/Module " + name + "/Turn Desired Volts", PID);
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
        drivePID.setPID(tunableDriveKP.get(), tunableDriveKI.get(), tunableDriveKD.get());
        turnPID.setPID(tunableTurnKP.get(), tunableTurnKI.get(), tunableTurnKD.get());

        driveFF.setKs(tunableDriveKS.get());
        driveFF.setKa(tunableDriveKA.get());
        driveFF.setKv(tunableDriveKV.get());

        turnFF.setKs(tunableTurnKS.get());
        turnFF.setKa(tunableTurnKA.get());
        turnFF.setKv(tunableTurnKV.get());

        setDriveSpeed(MetersPerSecond.of(desiredState.speedMetersPerSecond));
        setTurnPosition(desiredState.angle);
        updateInputs();
    }
}