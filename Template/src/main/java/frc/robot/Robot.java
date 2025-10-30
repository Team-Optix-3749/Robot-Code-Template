// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.AutoUtils;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.RobotType;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.ButtonBindings;
import frc.robot.utils.MiscUtils;

public class Robot extends LoggedRobot {
  public static Swerve swerve = new Swerve();

  Command autoCommand = null;

  public Robot() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0 -> Logger.recordMetadata("GitDirty", "All changes committed");
      case 1 -> Logger.recordMetadata("GitDirty", "Uncomitted changes");
      default -> Logger.recordMetadata("GitDirty", "Unknown");
    }

    RobotType robotType = MiscUtils.getRobotType();
    Logger.recordOutput("Robot Type", robotType);

    // Set up data receivers & replay source
    switch (robotType) {
      case REAL, SIM -> {
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
      }
      case REPLAY -> {
        // Replaying a log, set up replay source
        // setUseTiming(false); // Run as fast as possible
        // String logPath = LogFileUtil.findReplayLog();
        // Logger.setReplaySource(new WPILOGReader(logPath));
        // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
        // "_sim")));
      }
    }

    LoggedPowerDistribution.getInstance(RobotConfig.Can.PDH_ID, ModuleType.kRev);

    Logger.start();
  }

  @Override
  public void robotInit() {
    ButtonBindings.apply();
    AutoUtils.initAutoUtils();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void disabledExit() {

  }

  @Override
  public void autonomousInit() {
    autoCommand = AutoUtils.getChooser().selectedCommand();
    autoCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    if (autoCommand != null) {
      autoCommand.cancel();
      autoCommand = null;
    }
  }

  @Override
  public void teleopInit() {
    ButtonBindings.apply();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationInit() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
  }
}
