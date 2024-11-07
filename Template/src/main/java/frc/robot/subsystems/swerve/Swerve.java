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

  private ShuffleData<Double[]> odometryLog = new ShuffleData<Double[]>(
      this.getName(),
      "odometry",
      new Double[] { 0.0, 0.0, 0.0, 0.0 });

  private ShuffleData<Double[]> realStatesLog = new ShuffleData<Double[]>(
      this.getName(),
      "real states",
      new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });

  private ShuffleData<Double[]> desiredStatesLog = new ShuffleData<Double[]>(
      this.getName(),
      "desired states",
      new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });

  private ShuffleData<Double> velocityLog = new ShuffleData<Double>(this.getName(), "velocity", 0.0);
  private ShuffleData<Double> accelerationLog = new ShuffleData<Double>(this.getName(), "acceleration", 0.0);

  private ShuffleData<Double> yawLog = new ShuffleData<Double>(
      this.getName(),
      "yaw",
      0.0);

  private ShuffleData<Double> pitchLog = new ShuffleData<Double>(
      this.getName(),
      "pitch",
      0.0);

  private ShuffleData<Double> rollLog = new ShuffleData<Double>(
      this.getName(),
      "roll",
      0.0);

  private ShuffleData<Boolean> gyroConnectedLog = new ShuffleData<Boolean>(
      this.getName(),
      "gyro connected",

      false);
  private ShuffleData<Boolean> gyroCalibratingLog = new ShuffleData<Boolean>(
      this.getName(),
      "gyro calibrating",
      false);

  private ShuffleData<Double> headingLog = new ShuffleData<Double>(
      this.getName(),
      "heading",
      0.0);

  private double prevVelocity = 0;
  private boolean utilizeVision = true;

  public Swerve() {
    // if simulation
    if (Robot.isSimulation()) {
      gyro = new GyroSim();
      for (int i = 0; i < 4; i++) {
        modules[i] = new SwerveModule(i, new SwerveModuleSim());
      }
    }
    // if real
    else {
      // gyro = new NavX2Gyro();
      gyro = new PigeonGyro();
      for (int i = 0; i < 4; i++) {
        modules[i] = new SwerveModule(i, new SwerveModuleSparkMax(i));
      }
    }
    // pose estimator
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

    // put us on the field with a default orientation
    resetGyro();
    setOdometry(new Pose2d(1.33, 5.53, new Rotation2d(0)));

  }

  /**
   * Returns the coordinate and angular velocity of the robot
   * 
   * @return chassisSpeeds - ChasssisSpeeds object with a x, y, and rotaitonal
   *         velocity
   */
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

  /**
   * @return Returns direction the robot is facing as a Rotation2d object
   */
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

  /**
   * @return Returns coordinates and head the robot as a Pose2d object
   */
  public Pose2d getPose() {
    Pose2d estimatedPose = swerveDrivePoseEstimator.getEstimatedPosition();
    return new Pose2d(estimatedPose.getTranslation(), getRotation2d());
    // return new Pose2d(new Translation2d(2, 4.9), new Rotation2d(Math.PI/2));
  }

  /**
   * @return Returns max speed to be achieved by the robot based on telop or auto
   */
  public double getMaxDriveSpeed() {
    return DriverStation.isTeleopEnabled() ? SwerveConstants.DriveConstants.teleopMaxSpeedMetersPerSecond
        : SwerveConstants.DriveConstants.autoMaxSpeedMetersPerSecond;
  }

  /**
   * @return Returns max angular speed to be achieved by the robot based on telop
   *         or auto
   */
  public double getMaxAngularSpeed() {
    return DriverStation.isTeleopEnabled() ? SwerveConstants.DriveConstants.teleopMaxAngularSpeedMetersPerSecond
        : SwerveConstants.DriveConstants.autoMaxAngularSpeedMetersPerSecond;
  }

  /**
   * Turns a set of coordinate and angular velocities into module states, then
   * sets all modules to those states
   * 
   * @param chassisSpeeds - ChasssisSpeeds object with a x, y, and rotaitonal
   *                      velocity
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {

    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.driveKinematics.toSwerveModuleStates(
        chassisSpeeds);
    setModuleStates(moduleStates);

  }

  /**
   * takes a set of module states and sets individual modules to those states,
   * capping speeds to the maxmimum of the wheels
   * 
   * @param desiredStates - the individual module states coupled. FLD, FLT, FRD,
   *                      FRT, BLD, BLT, BRD, BRT
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates,
        DriveConstants.maxSpeedMetersPerSecond);

    modules[0].setDesiredState(desiredStates[0]);
    modules[1].setDesiredState(desiredStates[1]);
    modules[2].setDesiredState(desiredStates[2]);
    modules[3].setDesiredState(desiredStates[3]);

  }

  public void setBreakMode(boolean enable) {
    for (int i = 0; i < 4; i++) {
      modules[i].setBreakMode(enable);
    }
  }

  /**
   * Toggles whether or not vision updates odometry
   * 
   * @param utilize - whether or not to use vision updates on odometery, true is
   *                yes
   */
  public void setUtilizeVision(boolean utilize) {
    utilizeVision = utilize;
  }

  /**
   * Manually sets our odometry position
   * 
   * @param pose - Pose2d object of what to set our position to
   */
  public void setOdometry(Pose2d pose) {
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

  /**
   * Updates our odometry position based on module encoders. Ran in Periodic
   */
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

  /**
   * Updates our odometry position based on vision. Called by vision subsystem
   */
  public void visionUpdateOdometry(Pose2d pose, double timestamp) {
    if (utilizeVision) {
      SmartDashboard.putBoolean("use vision", utilizeVision);
      swerveDrivePoseEstimator.addVisionMeasurement(pose,
          timestamp);
    }
  }

  /**
   * Sets voltage to all swerve motors to 0
   */
  public void stopModules() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  /**
   * When called, makes the robot's current direction "forward"
   */
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

  @Override
  public void periodic() {
    gyro.updateData(gyroData);
    updateOdometry();

    // periodic method for individual modules
    for (int i = 0; i < 4; i++) {
      modules[i].periodic();
    }

    // logging of our module states
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

    // odometry logging
    odometryLog.set(
        new Double[] {
            getPose().getX(),
            getPose().getY(),
            getPose().getRotation().getDegrees()
        });

    // gyro logging
    yawLog.set(gyroData.yawDeg);
    pitchLog.set(gyroData.pitchDeg);
    rollLog.set(gyroData.rollDeg);
    gyroConnectedLog.set(gyroData.isConnected);
    gyroCalibratingLog.set(gyroData.isCalibrating);
    headingLog.set(getRotation2d().getDegrees());

    // velocity and acceleration logging
    double robotVelocity = Math.hypot(getChassisSpeeds().vxMetersPerSecond,
        getChassisSpeeds().vyMetersPerSecond);
    velocityLog.set(robotVelocity);
    accelerationLog.set((robotVelocity - prevVelocity) / .02);
    prevVelocity = robotVelocity;

  }

}
