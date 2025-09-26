package frc.robot.subsystems.swerve;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.util.Units;

public class Swerve extends SubsystemBase {
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive swerveDrive;

  public Swerve() {
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory)
          .createSwerveDrive(SwerveConstants.DriveConstants.maxSpeedMetersPerSecond);
    } catch (Exception e) {
      e.printStackTrace();

      System.out.println(
          "Could not parse swerve drive. Check that the swerve JSON files have been deployed to the robot.");
    }
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public Rotation2d getRotation() {
    return swerveDrive.getYaw();
  }
}
