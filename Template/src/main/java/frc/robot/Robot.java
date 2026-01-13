// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.AutoUtils;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.RobotType;
import frc.robot.subsystems.ExampleElevator.ExampleElevator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.config.ButtonBindings;
import frc.robot.utils.MiscUtils;

public class Robot extends LoggedRobot {
  public static Swerve swerve = new Swerve();
  public static ExampleElevator exampleElevator = new ExampleElevator();

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
    Logger.recordOutput("Replay Mode", MiscUtils.isReplay());

    // Set up data receivers & replay source
    switch (robotType) {
      case REAL -> {
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter());
      }
      case SIM -> {
        if (MiscUtils.isReplay()) {
          setUseTiming(false);
          String logPath = LogFileUtil.findReplayLog();
          Logger.setReplaySource(new WPILOGReader(logPath));
          Logger.addDataReceiver(new NT4Publisher());
          Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
              "_replay")));
        } else {
          Logger.addDataReceiver(new NT4Publisher());
          Logger.addDataReceiver(new WPILOGWriter());
        }
      }
    }

    LoggedPowerDistribution.getInstance(RobotConfig.CAN.PDH_ID, ModuleType.kRev);

    Logger.registerURCL(URCL.startExternal());
    Logger.start();
  }

  @Override
  public void robotInit() {
    ButtonBindings.apply();
    AutoUtils.initAutoUtils();
    DriverStation.silenceJoystickConnectionWarning(true);
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
    AutoUtils.runSelectedCommand();
    // Dont use ^ anymore. See AutoUtils.setupAutoTrigger()

    // In here should be just any special setup needed before auto starts
    // For example, if we do piece detection, maybe we would want to select a
    // different preset
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {

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
