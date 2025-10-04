// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoUtils;
import frc.robot.commands.auto.Autos;
import frc.robot.utils.JoystickIO;

public class RobotContainer {

  public RobotContainer() {
    JoystickIO.getButtonBindings();
    AutoUtils.initAuto();
  }

  public Command getAutonomousCommand() {
    // return new DriveStraight();
    return Autos.getChairGame();

    // return null;
  }
}
