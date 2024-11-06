package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.SwerveModuleIO.ModuleData;
import frc.robot.utils.ShuffleData;

public class SwerveModule {

    private double index;
    private String name;
    private SwerveModuleState desiredState = new SwerveModuleState();
    private final PIDController turningPidController;
    private final PIDController drivingPidController;
    private final SimpleMotorFeedforward drivingFeedFordward;

    private ModuleData moduleData = new ModuleData();
    private SwerveModuleIO moduleIO;

    // private double previousSetpointVelocity = 0;

    private ShuffleData<Double> driveSpeed;
    private ShuffleData<Double> drivePosition;
    private ShuffleData<Double> driveTemp;
    private ShuffleData<Double> driveVolts;
    private ShuffleData<Double> driveCurrent;

    private ShuffleData<Double> turningSpeed;
    private ShuffleData<Double> turningPosition;
    private ShuffleData<Double> turningTemp;
    private ShuffleData<Double> turningVolts;
    private ShuffleData<Double> turningCurrent;

    public SwerveModule(int i, SwerveModuleIO SwerveModule) {
        index = i;
        if (index == 0) {
            name = "FL Module";
        } else if (index == 1) {
            name = "FR Module";
        } else if (index == 2) {
            name = "BL Module";
        } else if (index == 3) {
            name = "BR Module";
        }

        moduleIO = SwerveModule;

        drivingPidController = new PIDController(ModuleConstants.kPDriving, 0, 0);
        drivingFeedFordward = new SimpleMotorFeedforward(ModuleConstants.kSDriving,
                ModuleConstants.kVDriving);
        turningPidController = new PIDController(ModuleConstants.kPturning, 0, ModuleConstants.kDTurning);
        turningPidController.enableContinuousInput(0, 2 * Math.PI);

        // Tab, name, data
        driveSpeed = new ShuffleData<>("swerve/" + name, name + " drive speed", moduleData.driveVelocityMPerSec);
        drivePosition = new ShuffleData<>("swerve/" + name, name + " drive position",
        moduleData.driveVelocityMPerSec);
        driveTemp = new ShuffleData<>("swerve/" + name, name + " drive temp",
        moduleData.driveVelocityMPerSec);
        driveVolts = new ShuffleData<>("swerve/" + name, name + " drive volts",
        moduleData.driveVelocityMPerSec);
        driveCurrent = new ShuffleData<>("swerve/" + name, name + " drive current", moduleData.driveVelocityMPerSec);

        turningSpeed = new ShuffleData<>("swerve/" + name, name + " turning speed",
        moduleData.driveVelocityMPerSec);
        turningPosition = new ShuffleData<>("swerve/" + name, name + " turning position",
                moduleData.driveVelocityMPerSec);
        turningTemp = new ShuffleData<>("swerve/" + name, name + " turning temp",
        moduleData.driveVelocityMPerSec);
        turningVolts = new ShuffleData<>("swerve/" + name, name + " turning volts",
        moduleData.driveVelocityMPerSec);
        turningCurrent = new ShuffleData<>("swerve/" + name, name + " turning current", moduleData.turnCurrentAmps);
    }

    public String getName() {
        return name;
    }

    public SwerveModuleState getState() {

        return new SwerveModuleState(
                moduleData.driveVelocityMPerSec,
                new Rotation2d(moduleData.turnAbsolutePositionRad));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(moduleData.drivePositionM, new Rotation2d(moduleData.turnAbsolutePositionRad));
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(SwerveModuleState state) {

        state = SwerveModuleState.optimize(state, getState().angle);

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            state.speedMetersPerSecond = 0;
        }

        this.desiredState = state;

        setDriveSpeed(state.speedMetersPerSecond);
        setTurnPosition(state.angle.getRadians());

    }

    public void setDriveSpeed(double speedMetersPerSecond) {
        double drive_volts = 0;

        drive_volts = drivingFeedFordward.calculate(speedMetersPerSecond)
                + drivingPidController.calculate(moduleData.driveVelocityMPerSec, speedMetersPerSecond);

        // drive_volts += Robot.kSdata.get() * Math.signum(speedMetersPerSecond);
        // drive_volts += Robot.kVdata.get() *(speedMetersPerSecond);
        // drive_volts += Robot.kAdata.get() * (speedMetersPerSecond -
        // previousSetpointVelocity) / 0.02;
        // previousSetpointVelocity = speedMetersPerSecond;

        // SmartDashboard.putNumber("acceleration", (speedMetersPerSecond -
        // previousSetpointVelocity) / 0.02);

     
        setDriveVoltage(drive_volts);

    }

    public void setTurnPosition(double positionRad) {
        double turning_volts = turningPidController.calculate(moduleData.turnAbsolutePositionRad,
                positionRad);
        // Make a drive PID Controller
        setTurnVoltage(turning_volts);
    }

    public void setDriveVoltage(double volts) {
        moduleIO.setDriveVoltage(volts);

    }

    public void setTurnVoltage(double volts) {
        moduleIO.setTurnVoltage(volts);
    }

    public void stop() {
        setDriveVoltage(0);
        setTurnVoltage(0);
    }

    public ModuleData getModuleData() {
        return moduleData;
    }

    public void setBreakMode(boolean enabled) {
        moduleIO.setDriveBrakeMode(enabled);
        moduleIO.setTurningBrakeMode(enabled);

    }

    // called within the swerve subsystem's periodic
    public void periodic() {
        moduleIO.updateData(moduleData);

        driveSpeed.set(moduleData.driveVelocityMPerSec);
        drivePosition.set(moduleData.drivePositionM);
        driveTemp.set(moduleData.driveTempCelcius);
        driveVolts.set(moduleData.driveAppliedVolts);
        driveCurrent.set(moduleData.driveCurrentAmps);

        turningSpeed.set(moduleData.turnVelocityRadPerSec);
        turningPosition.set(moduleData.turnAbsolutePositionRad);
        turningTemp.set(moduleData.turnTempCelcius);
        turningVolts.set(moduleData.turnAppliedVolts);
        turningCurrent.set(moduleData.turnCurrentAmps);

    }
}
