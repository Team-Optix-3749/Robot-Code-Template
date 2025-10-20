package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;


import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.gyro.GyroDataAutoLogged;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroSim;
import frc.robot.subsystems.swerve.gyro.PigeonGyro;
import frc.robot.subsystems.swerve.sim.SwerveModuleSim;
import frc.robot.config.RobotConfig.RobotType;
import frc.robot.utils.MiscUtils;
import frc.robot.subsystems.swerve.SwerveConfig.Control;
import frc.robot.subsystems.swerve.SwerveConfig.Drivetrain;
import frc.robot.subsystems.swerve.SwerveConfig.PoseEstimator;
import frc.robot.subsystems.swerve.real.*;

/**
 * Subsystem class for swerve drive, used to manage four swerve
 * modules and set their states. Also includes a pose estimator,
 * gyro, and logging information.
 * 
 * Rotation standard: everything is relative to blue alliance. 0 is
 * from blue alliance wall, counter-clockwise positive.
 */
public class Swerve extends SubsystemBase {
  private final SwerveModule[] modules = new SwerveModule[4];

  private final GyroIO gyro;
  private final GyroDataAutoLogged gyroData = new GyroDataAutoLogged();

  private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();

  private final ProfiledPIDController autoTranslateController = new ProfiledPIDController(
      SwerveConfig.Control.drivetrainTranslatePID[0], SwerveConfig.Control.drivetrainTranslatePID[1],
      SwerveConfig.Control.drivetrainTranslatePID[2], Control.translateControllerConstants);
  private final ProfiledPIDController autoTurnController = new ProfiledPIDController(
      SwerveConfig.Control.drivetrainTurnPID[0], SwerveConfig.Control.drivetrainTurnPID[1],
      SwerveConfig.Control.drivetrainTurnPID[2], Control.turnControllerConstants);

  public Swerve() {
    var robotType = MiscUtils.getRobotType();
    Logger.recordOutput("Swerve/RobotType", robotType.name());
    gyro = robotType == RobotType.SIM ? new GyroSim(gyroData) : new PigeonGyro(gyroData);

    SwerveModuleType moduleType = robotType == RobotType.SIM ? SwerveModuleType.SIM : SwerveModuleType.SPARK;
    for (int i = 0; i < modules.length; i++) {
      modules[i] = new SwerveModule(i, moduleType);
    }

    // Initialize pose estimator
    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
        Drivetrain.driveKinematics,
        new Rotation2d(),
        collectModulePositions(),
        PoseEstimator.INITIAL_POSE,
        VecBuilder.fill(
            PoseEstimator.STATE_STD_DEVS[0],
            PoseEstimator.STATE_STD_DEVS[1],
            PoseEstimator.STATE_STD_DEVS[2]),
        VecBuilder.fill(
            PoseEstimator.VISION_STD_DEVS[0],
            PoseEstimator.VISION_STD_DEVS[1],
            PoseEstimator.VISION_STD_DEVS[2]));

    // Configure auto PID controllers
    // autoZController.enableContinuousInput(-Math.PI, Math.PI);
    // autoZController.setIZone(AutoConstants.turnIZone);
    // autoZController.setTolerance(AutoConstants.turnToleranceRad / 2);
    // autoXController.setTolerance(AutoConstants.driveToleranceMeters / 3);
    // autoYController.setTolerance(AutoConstants.driveToleranceMeters / 3);
    // autoXController.setIZone(AutoConstants.driveIZone);
    // autoYController.setIZone(AutoConstants.driveIZone);

    autoTurnController.enableContinuousInput(-Math.PI, Math.PI);
    resetGyro();
  }

  public SwerveDrivePoseEstimator getSwerveDrivePoseEstimator() {
    return swerveDrivePoseEstimator;
  }

  /**
   * @return robot-relative chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }

    return Drivetrain.driveKinematics.toChassisSpeeds(states);
  }

  /**
   * @return Current rotation
   */
  public Rotation2d getRotation() {
    return swerveDrivePoseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * @return Current pose on the field
   */
  public Pose2d getPose() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  /**
   * @return Max speed in meters per second
   */
  public double getMaxDriveSpeed() {
    return DriverStation.isTeleopEnabled() ? Control.maxSpeedMPS
        : Control.maxSpeedMPS;
  }

  /**
   * @return Max angular speed in radians per second
   */
  public double getMaxAngularSpeed() {
    return Control.maxAngularSpeedRadsPS;
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return swerveDrivePoseEstimator;
  }

  /**
   * Sets individual module states with desaturation.
   * 
   * @param desiredStates Array of desired module states
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, getMaxDriveSpeed());
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(desiredStates[i]);
    }
  }

  /**
   * Sets brake mode for all modules.
   * 
   * @param enable True to enable brake mode, false for coast
   */
  public void setBrakeMode(boolean enable) {
    for (SwerveModule module : modules) {
      module.setBrakeMode(enable);
    }
  }

  public void setOdometry(Pose2d pose) {
    Rotation2d gyroHeading = Rotation2d.fromRadians(gyroData.orientation.getZ());
    swerveDrivePoseEstimator.resetPosition(
        gyroHeading,
        collectModulePositions(),
        pose);
  }

  public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
    desiredChassisSpeeds = chassisSpeeds;
  }

  public void driveToSample(SwerveSample sample) {
    double vx = sample.vx + autoTranslateController.calculate(
        swerveDrivePoseEstimator.getEstimatedPosition().getX(), sample.x);
    double vy = sample.vy + autoTranslateController.calculate(
        swerveDrivePoseEstimator.getEstimatedPosition().getY(), sample.y);
    double targetAngle = MiscUtils.isRedAlliance()
        ? sample.heading + Math.PI
        : sample.heading;
    double currentAngle = swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getRadians();
    double omega = sample.omega + autoTurnController.calculate(currentAngle, targetAngle);

    ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);

    driveFieldRelative(speeds);
  }

  public void driveToPose(Pose2d pose) {
    SwerveSample sample = new SwerveSample(0, pose.getX(), pose.getY(), pose.getRotation().getRadians(), 0, 0, 0, 0, 0,
        0, new double[4], new double[4]);

    driveToSample(sample);
  }

  public void lockModules() {
    Rotation2d lockAngle = Rotation2d.fromDegrees(-45);
    for (SwerveModule module : modules) {
      module.setDesiredState(new SwerveModuleState(0, lockAngle));
      lockAngle = lockAngle.plus(Rotation2d.fromDegrees(90));
    }
  }

  public void stopModules() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  public void updateOdometry() {
    swerveDrivePoseEstimator.update(
        Rotation2d.fromRadians(gyroData.orientation.getZ()),
        collectModulePositions());
  }

  public void resetGyro() {
    gyro.reset();

    for (SwerveModule module : modules) {
      module.syncEncoderPosition();
    }

    Rotation2d targetRotation = MiscUtils.isRedAlliance()
        ? Rotation2d.fromDegrees(180)
        : new Rotation2d();

    swerveDrivePoseEstimator.resetPosition(
        new Rotation2d(),
        collectModulePositions(),
        new Pose2d(swerveDrivePoseEstimator.getEstimatedPosition().getTranslation(), targetRotation));
  }

  private SwerveModulePosition[] collectModulePositions() {
    return new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
    };
  }

  /**
   * Logs all swerve telemetry data.
   */
  private void logData() {
    SwerveModuleState[] moduleDesiredStates = new SwerveModuleState[4];
    SwerveModuleState[] moduleRealStates = new SwerveModuleState[4];

    for (int i = 0; i < modules.length; i++) {
      moduleDesiredStates[i] = modules[i].getDesiredState();
      moduleRealStates[i] = modules[i].getState();
    }

    Logger.recordOutput("Swerve/RealStates", moduleRealStates);
    Logger.recordOutput("Swerve/DesiredStates", moduleDesiredStates);

    Logger.recordOutput("Swerve/Pose", getPose());

    Logger.recordOutput("Swerve/RealChassisSpeeds", getChassisSpeeds());
    Logger.recordOutput("Swerve/DesiredChassisSpeeds", desiredChassisSpeeds);

    // Current command
    String currentCommand = this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName();
    Logger.recordOutput("Swerve/CurrentCommand", currentCommand);
  }

  @Override
  public void periodic() {
    gyro.updateData();
    Logger.processInputs("Swerve/GyroData", gyroData);

    SwerveModuleState[] desiredStates = Drivetrain.driveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
    setModuleStates(desiredStates);

    for (SwerveModule module : modules) {
      module.periodic();
    }

    updateOdometry();
    logData();
  }
}