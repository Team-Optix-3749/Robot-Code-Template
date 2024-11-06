// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.GyroIO.GyroData;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.real.*;
import frc.robot.subsystems.swerve.sim.*;
import frc.robot.utils.*;

/***
 * @author Noah Simon
 * @author Rohin Sood
 * @author Raadwan Masum
 * @author Harkirat
 *
 *         Subsystem class for swerve drive, used to manage four swerve modules
 *         and set their states. Also includes a pose estimator, gyro, and
 *         logging information
 */
public class Swerve extends SubsystemBase {

  private SwerveModule[] modules = new SwerveModule[4];

  private GyroIO gyro;
  private GyroData gyroData = new GyroData();
  // equivilant to a odometer, but also intakes vision
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  private boolean isEnabled = false;

  private ShuffleData<Double[]> odometryLog = new ShuffleData<Double[]>(
      "swerve",
      "odometry",
      new Double[] { 0.0, 0.0, 0.0, 0.0 });
  private ShuffleData<Double[]> realStatesLog = new ShuffleData<Double[]>(
      "swerve",
      "real states",
      new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
  private ShuffleData<Double[]> desiredStatesLog = new ShuffleData<Double[]>(
      "swerve",
      "desired states",
      new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
  private ShuffleData<Double> yawLog = new ShuffleData<Double>(
      "swerve",
      "yaw",
      0.0);

  private ShuffleData<Boolean> gyroConnectedLog = new ShuffleData<Boolean>(
      "swerve",
      "gyro connected",

      false);
  private ShuffleData<Boolean> gyroCalibratingLog = new ShuffleData<Boolean>(
      "swerve",
      "gyro calibrating",
      false);

  private ShuffleData<Double> headingLog = new ShuffleData<Double>(
      "swerve",
      "heading",
      0.0);

  private double prevVelocity = 0;
  private boolean utilizeVision = true;

  public Swerve() {
    if (Robot.isSimulation()) {
      gyro = new GyroSim();
      for (int i = 0; i < 4; i++) {
        modules[i] = new SwerveModule(i, new SwerveModuleSim());
      }
    } else {
      // real swerve module instatiation here
      /* gyro = new NavX2Gyro(); */
      gyro = new PigeonGyro();
      for (int i = 0; i < 4; i++) {
        modules[i] = new SwerveModule(i, new SwerveModuleSparkMax(i));
      }
    }

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.driveKinematics,
        new Rotation2d(0),
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        },
        new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
        VecBuilder.fill(0.04, 0.04, 0.00),
        VecBuilder.fill(0.965, 0.965, 5000));

    if (Robot.isSimulation()) {
      // resetOdometry(new Pose2d(new Translation2d(1, 1), new
      // Rotation2d(Units.degreesToRadians(270))));
    }

    resetGyro();
    resetOdometry(new Pose2d(1.33, 5.53, new Rotation2d(0)));

  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.driveKinematics.toSwerveModuleStates(
        chassisSpeeds);
    // take shortest path to destination
    if (DriverStation.isTeleopEnabled()) {

      SwerveDriveKinematics.desaturateWheelSpeeds(
          moduleStates,
          DriveConstants.teleopMaxSpeedMetersPerSecond);
    } else {

      SwerveDriveKinematics.desaturateWheelSpeeds(
          moduleStates,
          DriveConstants.maxSpeedMetersPerSecond);
    }
    // 6. Output each module states to wheels

    setModuleStates(moduleStates);

  }

  public void setUtilizeVision(boolean utilize) {
    utilizeVision = utilize;
  }

  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        DriveConstants.driveKinematics.toChassisSpeeds(states),
        getRotation2d());
    return speeds;
  }

  public Rotation2d getRotation2d() {
    Rotation2d rotation = swerveDrivePoseEstimator
        .getEstimatedPosition()
        .getRotation();
    // return rotation;
    double heading = rotation.getDegrees();

    if (heading < 0) {
      heading += 360;
    }
    return new Rotation2d(heading / 180 * Math.PI);
  }

  public Pose2d getPose() {
    Pose2d estimatedPose = swerveDrivePoseEstimator.getEstimatedPosition();
    return new Pose2d(estimatedPose.getTranslation(), getRotation2d());
    // return new Pose2d(new Translation2d(2, 4.9), new Rotation2d(Math.PI/2));
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return swerveDrivePoseEstimator;
  }

  public void resetOdometry(Pose2d pose) {
    // convert to -pi to pi
    Rotation2d gyroHeading = new Rotation2d(gyroData.yawDeg / 180 * Math.PI);
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

  public void updateOdometry() {
    // convert to -pi to pi
    Rotation2d gyroHeading = Rotation2d.fromRadians(
        MathUtil.angleModulus(Units.degreesToRadians(gyroData.yawDeg)));

    swerveDrivePoseEstimator.update(
        gyroHeading,
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        });
  }

  public void visionUpdateOdometry(Pose2d pose, double timestamp) {
    if (utilizeVision) {
      SmartDashboard.putBoolean("use vision", utilizeVision);
      swerveDrivePoseEstimator.addVisionMeasurement(pose,
          timestamp);
    }
  }

 

  public void stopModules() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates,
        DriveConstants.maxSpeedMetersPerSecond);

    modules[0].setDesiredState(desiredStates[0]);
    modules[1].setDesiredState(desiredStates[1]);
    modules[2].setDesiredState(desiredStates[2]);
    modules[3].setDesiredState(desiredStates[3]);

  }

  public double getVerticalTilt() {
    return gyroData.pitchDeg;
  }

  public void resetGyro() {
    gyro.resetGyro();
    if (UtilityFunctions.isRedAlliance()) {

      swerveDrivePoseEstimator.resetPosition(new Rotation2d(), new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
      }, new Pose2d(swerveDrivePoseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      swerveDrivePoseEstimator.resetPosition(new Rotation2d(), new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
      }, new Pose2d(swerveDrivePoseEstimator.getEstimatedPosition().getTranslation(), new Rotation2d()));
    }
  }

  public double totalAcceleration = 0;

  @Override
  public void periodic() {
    gyro.updateData(gyroData);
    updateOdometry();

    for (int i = 0; i < 4; i++) {
      modules[i].periodic();
    }

    Double[] realStates = {
        modules[0].getState().angle.getDegrees(),
        modules[0].getState().speedMetersPerSecond,
        modules[1].getState().angle.getDegrees(),
        modules[1].getState().speedMetersPerSecond,
        modules[2].getState().angle.getDegrees(),
        modules[2].getState().speedMetersPerSecond,
        modules[3].getState().angle.getDegrees(),
        modules[3].getState().speedMetersPerSecond
    };

    Double[] desiredStates = {
        modules[0].getDesiredState().angle.getDegrees(),
        modules[0].getDesiredState().speedMetersPerSecond,
        modules[1].getDesiredState().angle.getDegrees(),
        modules[1].getDesiredState().speedMetersPerSecond,
        modules[2].getDesiredState().angle.getDegrees(),
        modules[2].getDesiredState().speedMetersPerSecond,
        modules[3].getDesiredState().angle.getDegrees(),
        modules[3].getDesiredState().speedMetersPerSecond
    };

    realStatesLog.set(realStates);
    desiredStatesLog.set(desiredStates);

    odometryLog.set(
        new Double[] {
            getPose().getX(),
            getPose().getY(),
            getPose().getRotation().getDegrees()
        });

    yawLog.set(gyroData.yawDeg);
    // pitchLog.set(gyroData.pitchDeg);
    // rollLog.set(gyroData.rollDeg);
    gyroConnectedLog.set(gyroData.isConnected);
    gyroCalibratingLog.set(gyroData.isCalibrating);
    headingLog.set(getRotation2d().getDegrees());

    double robotVelocity = Math.sqrt(Math.pow(getChassisSpeeds().vxMetersPerSecond, 2) +
        Math.pow(getChassisSpeeds().vyMetersPerSecond, 2));

    SmartDashboard.putNumber("robot velocity", robotVelocity);

    SmartDashboard.putNumber("robot acceleration", (robotVelocity - prevVelocity) / .02);
    prevVelocity = robotVelocity;

    boolean driverStationStatus = DriverStation.isEnabled();
    if (driverStationStatus && !isEnabled) {
      isEnabled = driverStationStatus;
      modules[0].setBreakMode(true);
      ;
    }
    if (!driverStationStatus && isEnabled) {
      modules[0].setBreakMode(false);
      ;
      isEnabled = driverStationStatus;
    }

  }

}
