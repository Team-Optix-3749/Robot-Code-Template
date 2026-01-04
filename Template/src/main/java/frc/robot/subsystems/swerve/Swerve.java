package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.gyro.GyroDataAutoLogged;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroSim;
import frc.robot.subsystems.swerve.gyro.PigeonGyro;
import frc.robot.commands.auto.AutoUtils;
import frc.robot.config.RobotConfig.RobotType;
import frc.robot.config.AutoConfig;
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

  private PIDController xController = new PIDController(AutoConfig.Drive.KP, AutoConfig.Drive.KI, AutoConfig.Drive.KD);
  private PIDController yController = new PIDController(AutoConfig.Drive.KP, AutoConfig.Drive.KI, AutoConfig.Drive.KD);
  private PIDController turnController = new PIDController(AutoConfig.Turn.KP, AutoConfig.Turn.KI, AutoConfig.Turn.KD);

  private Pose2d positionSetpoint = new Pose2d();
  private Pose2d velocitySetpoint = new Pose2d();

  private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  private boolean isOTF = false;

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

  public void setIsOTF(boolean otf) {
    isOTF = otf;
  }

  public boolean getIsOTF() {
    return isOTF;
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

  public void followSample(Pose2d positions, Pose2d velocities) {
    positionSetpoint = positions;
    velocitySetpoint = velocities;

    double xPID = xController.calculate(getPose().getX(), positions.getX());
    xPID = MathUtil.clamp(xPID, -1, 1);
    if (xController.atSetpoint()) {
      SmartDashboard.putBoolean("xAtSetpoint", true);
      xPID = 0;
    }

    double yPID = yController.calculate(getPose().getY(), positions.getY());
    yPID = MathUtil.clamp(yPID, -1, 1);

    if (yController.atSetpoint()) {
      SmartDashboard.putBoolean("yAtSetpoint", true);
      yPID = 0;
    }

    double turnPID = turnController.calculate(getPose().getRotation().getRadians(),
        positions.getRotation().getRadians());
    // turnPID = MathUtil.clamp(turnPID, -Math.PI/2, Math.PI/2);

    if (turnController.atSetpoint()) {
      turnPID = 0;
    }
    Logger.recordOutput("Swerve/auto/turn PID", turnPID);
    Logger.recordOutput("Swerve/auto/x PID", xPID);
    Logger.recordOutput("Swerve/auto/y PID", yPID);

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(
            xPID + velocities.getX(),
            yPID + velocities.getY(),
            turnPID
                + velocities.getRotation().getRadians()),
        getPose().getRotation());
    logSetpoints(positions, velocities);
    Logger.recordOutput("Swerve/auto/velocity hypt", Math.sqrt(
        speeds.vxMetersPerSecond * speeds.vxMetersPerSecond + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond));
    driveFieldRelative(speeds);
  }

  public void logSetpoints(Pose2d position, Pose2d velocity) {
    logSetpoints(position.getX(), velocity.getX(), 0, position.getY(), velocity.getY(), 0,
        position.getRotation().getRadians(), velocity.getRotation().getRadians(), 0);

  }

  public void logSetpoints(double posX, double velX, double accX, double posY, double velY, double accY, double heading,
      double omega, double alpha) {
    // setpoint logging for automated driving
    double[] positions = new double[] { posX, posY, heading };
    Logger.recordOutput("Swerve/auto/position setpoint", positions);
    Transform2d poseDiff = new Pose2d(posX, posY, new Rotation2d(heading)).minus(getPose());
    Logger.recordOutput("Swerve/auto/position error", poseDiff);

    Double[] velocities = new Double[] { velX, velY, omega };
    double velocity = 0;
    velocity += Math.pow(velocities[0], 2);
    velocity += Math.pow(velocities[1], 2);

    velocity = Math.sqrt(velocity);
    Logger.recordOutput("Swerve/auto/setpoint velocity", velocity);
    Logger.recordOutput("Swerve/auto/setpoint rotational velocity", velocities[2]);
    velocity = velocities[2];
    Logger.recordOutput("Swerve/auto/velocity hypt", Math.sqrt(
        velX * velX + velY * velY));

    Double[] accelerations = new Double[] { accX, accY, alpha };
    double acceleration = 0;
    acceleration += Math.pow(accelerations[0], 2);
    acceleration += Math.pow(accelerations[1], 2);

    acceleration = Math.sqrt(acceleration);
    Logger.recordOutput("Swerve/auto/setpoint acceleration", acceleration);
    Logger.recordOutput("Swerve/auto/setpoint rotational acceleration", accelerations[2]);
  }

  public void followSample(SwerveSample sample, boolean isFlipped) {

    // ternaries are for x-axis flipping

    double xPos = sample.x;
    double xVel = sample.vx;
    double xAcc = sample.ax;

    double yPos = isFlipped ? AutoUtils.flipper.flipY(sample.y) : sample.y;
    double yVel = isFlipped ? -sample.vy : sample.vy;
    double yAcc = isFlipped ? -sample.ay : sample.ay;

    double heading = isFlipped ? new Rotation2d(Math.PI - sample.heading).rotateBy(new Rotation2d(Math.PI)).getRadians()
        : sample.heading;
    double omega = isFlipped ? -sample.omega : sample.omega;
    double alpha = isFlipped ? -sample.alpha : sample.alpha;

    positionSetpoint = new Pose2d(xPos, yPos, new Rotation2d(heading));
    velocitySetpoint = new Pose2d(xVel, yVel, new Rotation2d(omega));

    followSample(positionSetpoint, velocitySetpoint);
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