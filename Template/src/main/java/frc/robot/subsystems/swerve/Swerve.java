package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
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
import frc.robot.utils.MiscConfig;
import frc.robot.utils.UtilityFunctions;
import frc.robot.subsystems.swerve.SwerveConfig.Control;
import frc.robot.subsystems.swerve.SwerveConfig.Drivetrain;
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

  public Swerve() {
    // Initialize modules and gyro
    if (MiscConfig.ROBOT_TYPE == MiscConfig.RobotType.SIM) {
      gyro = new GyroSim();
      for (int i = 0; i < 4; i++) {
        modules[i] = new SwerveModule(i, new SwerveModuleSim(i));
      }
    } else {
      gyro = new PigeonGyro();
      for (int i = 0; i < 4; i++) {
        modules[i] = new SwerveModule(i, new SwerveModuleSpark(i));
      }
    }

    // Initialize pose estimator
    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
        Drivetrain.driveKinematics,
        new Rotation2d(0),
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        },
        new Pose2d(new Translation2d(5.773, 3.963), Rotation2d.fromDegrees(180)),
        VecBuilder.fill(0.045, 0.045, 0.004), // 6328's 2024 numbers with factors of 1.5x, 1.5x, 2x
        VecBuilder.fill(1e-6, 1e-6, 1e-6));

    // Configure auto PID controllers
    // autoZController.enableContinuousInput(-Math.PI, Math.PI);
    // autoZController.setIZone(AutoConstants.turnIZone);
    // autoZController.setTolerance(AutoConstants.turnToleranceRad / 2);
    // autoXController.setTolerance(AutoConstants.driveToleranceMeters / 3);
    // autoYController.setTolerance(AutoConstants.driveToleranceMeters / 3);
    // autoXController.setIZone(AutoConstants.driveIZone);
    // autoYController.setIZone(AutoConstants.driveIZone);
  }

  public SwerveDrivePoseEstimator getSwerveDrivePoseEstimator() {
    return swerveDrivePoseEstimator;
  }

  /**
   * @return Field-relative chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        Drivetrain.driveKinematics.toChassisSpeeds(states),
        getRotation());
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
    return DriverStation.isTeleopEnabled() ? Control.teleopMaxSpeedMetersPerSecond
        : Control.autoMaxSpeedMetersPerSecond;
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

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] moduleStates = Drivetrain.driveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);

    desiredChassisSpeeds = chassisSpeeds;
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
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        },
        pose);
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
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        });
  }

  public void resetGyro() {
    System.out.println("Resetting gyro");

    gyro.reset();

    for (SwerveModule module : modules) {
      module.syncEncoderPosition();
    }

    Rotation2d targetRotation = UtilityFunctions.isRedAlliance()
        ? Rotation2d.fromDegrees(180)
        : new Rotation2d();

    swerveDrivePoseEstimator.resetPosition(
        new Rotation2d(),
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        },
        new Pose2d(swerveDrivePoseEstimator.getEstimatedPosition().getTranslation(), targetRotation));
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

    Logger.recordOutput("Swerve/DesiredStates", moduleRealStates);
    Logger.recordOutput("Swerve/RealStates", moduleRealStates);

    Logger.recordOutput("Swerve/Pose", getPose());
    Logger.recordOutput("Swerve/Heading", getRotation());

    Logger.recordOutput("Swerve/DesiredChassisSpeeds", desiredChassisSpeeds);
    Logger.recordOutput("Swerve/RealChassisSpeeds", getChassisSpeeds());

    // Current command
    String currentCommand = this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName();
    Logger.recordOutput("Swerve/CurrentCommand", currentCommand);
  }

  @Override
  public void periodic() {
    gyro.update(gyroData);
    Logger.processInputs("Swerve/GyroData", gyroData);

    for (SwerveModule module : modules) {
      module.periodic();
    }

    updateOdometry();
    logData();
  }
}