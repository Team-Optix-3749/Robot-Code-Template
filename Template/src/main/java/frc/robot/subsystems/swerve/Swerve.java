package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.gyro.GyroDataAutoLogged;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroSim;
import frc.robot.subsystems.swerve.gyro.PigeonGyro;
import frc.robot.config.RobotConfig.RobotType;
import frc.robot.config.SwerveConfig;
import frc.robot.utils.MiscUtils;
import frc.robot.config.SwerveConfig.Control;
import frc.robot.config.SwerveConfig.Drivetrain;
import frc.robot.config.SwerveConfig.PoseEstimator;

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
  private final SwerveModulePosition[] modulePositionsBuffer = new SwerveModulePosition[4];

  private final GyroIO gyro;
  private final GyroDataAutoLogged gyroData = new GyroDataAutoLogged();

  private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();

  private double lastEncoderSyncTime = -1.0;

  private final ProfiledPIDController autoXController = new ProfiledPIDController(
      SwerveConfig.Control.TRANSLATE_PID[0], SwerveConfig.Control.TRANSLATE_PID[1],
      SwerveConfig.Control.TRANSLATE_PID[2], Control.TRANSLATE_CONSTRAINTS);
  private final ProfiledPIDController autoYController = new ProfiledPIDController(
      SwerveConfig.Control.TRANSLATE_PID[0], SwerveConfig.Control.TRANSLATE_PID[1],
      SwerveConfig.Control.TRANSLATE_PID[2], Control.TRANSLATE_CONSTRAINTS);
  private final ProfiledPIDController autoTurnController = new ProfiledPIDController(
      SwerveConfig.Control.ROTATE_PID[0], SwerveConfig.Control.ROTATE_PID[1],
      SwerveConfig.Control.ROTATE_PID[2], Control.ROTATE_CONSTRAINTS);

  public Swerve() {
    RobotType robotType = MiscUtils.getRobotType();
    Logger.recordOutput("Swerve/RobotType", robotType.name());

    SwerveModuleType moduleType;

    switch (robotType) {
      case SIM:
        moduleType = SwerveModuleType.SIM;
        gyro = new GyroSim(gyroData);
        break;
      case REAL:
      default:
        moduleType = SwerveModuleType.SPARK;
        gyro = new PigeonGyro(gyroData);
        break;
    }

    for (int i = 0; i < modules.length; i++) {
      modules[i] = new SwerveModule(i, moduleType);
    }

    // Initialize pose estimator
    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
        Drivetrain.DRIVE_KINEMATICS,
        new Rotation2d(),
        getModulePositions(),
        PoseEstimator.INITIAL_POSE,
        PoseEstimator.STATE_STD_DEVS,
        PoseEstimator.VISION_STD_DEVS);

    resetGyro();

    // Configure auto PID controllers
    autoXController.reset(PoseEstimator.INITIAL_POSE.getX());
    autoYController.reset(PoseEstimator.INITIAL_POSE.getY());
    autoTurnController.enableContinuousInput(-Math.PI, Math.PI);
    autoTurnController.reset(PoseEstimator.INITIAL_POSE.getRotation().getRadians());

  }

  public SwerveDrivePoseEstimator getSwerveDrivePoseEstimator() {
    return swerveDrivePoseEstimator;
  }

  private SwerveModulePosition[] getModulePositions() {
    for (int i = 0; i < modules.length; i++) {
      modulePositionsBuffer[i] = modules[i].getPosition();
    }
    return modulePositionsBuffer;
  }

  /**
   * @return robot-relative chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }

    return Drivetrain.DRIVE_KINEMATICS.toChassisSpeeds(states);
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
    return Control.MAX_SPEED_MPS;
  }

  /**
   * @return Max angular speed in radians per second
   */
  public double getMaxAngularSpeed() {
    return Control.MAX_ANGULAR_SPEED_RADSS;
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
    for (int i = 0; i < modules.length; i++) {
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
        getModulePositions(),
        pose);
    resetAutoControllers(pose);
  }

  public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
    desiredChassisSpeeds = chassisSpeeds;
  }

  public void driveToSample(SwerveSample sample) {
    Pose2d pose = getPose();

    autoXController.setGoal(sample.x);
    autoYController.setGoal(sample.y);

    double vx = sample.vx + autoXController.calculate(pose.getX());
    double vy = sample.vy + autoYController.calculate(pose.getY());

    double targetAngle = MiscUtils.isRedAlliance()
        ? sample.heading + Math.PI
        : sample.heading;
    autoTurnController.setGoal(targetAngle);
    double omega = sample.omega + autoTurnController.calculate(pose.getRotation().getRadians());

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, pose.getRotation());

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
        getModulePositions());
  }

  public void syncEncoderPositions() {
    for (SwerveModule module : modules) {
      module.syncEncoderPosition();
    }
  }

  public void resetGyro() {
    gyro.reset();

    syncEncoderPositions();

    Rotation2d targetRotation = MiscUtils.isRedAlliance()
        ? Rotation2d.fromDegrees(180)
        : new Rotation2d();

    swerveDrivePoseEstimator.resetPosition(
        new Rotation2d(),
        getModulePositions(),
        new Pose2d(swerveDrivePoseEstimator.getEstimatedPosition().getTranslation(), targetRotation));
    resetAutoControllers(swerveDrivePoseEstimator.getEstimatedPosition());
  }

  private void resetAutoControllers(Pose2d pose) {
    autoXController.reset(pose.getX());
    autoYController.reset(pose.getY());
    autoTurnController.reset(pose.getRotation().getRadians());
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

    Logger.processInputs("Swerve/GyroData", gyroData);
  }

  @Override
  public void periodic() {
    gyro.updateData();
    updateOdometry();

    SwerveModuleState[] desiredStates = Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(desiredChassisSpeeds);
    setModuleStates(desiredStates);

    for (SwerveModule module : modules) {
      module.periodic();
    }

    double netMovement = Math.abs(desiredChassisSpeeds.vxMetersPerSecond)
        + Math.abs(desiredChassisSpeeds.vyMetersPerSecond)
        + Math.abs(desiredChassisSpeeds.omegaRadiansPerSecond);

    if (netMovement < 0.1 && (Timer.getTimestamp() - lastEncoderSyncTime) > 20.0) {
      syncEncoderPositions();
      lastEncoderSyncTime = Timer.getTimestamp();
    }

    logData();
  }
}