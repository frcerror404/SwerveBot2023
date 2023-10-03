// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Console;
import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Swerve extends SubsystemBase {
  SwerveDrive drive;
  
  /** Creates a new Swerve. */
  public Swerve() {
    try {
      drive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive();
    } catch (IOException ioException) {
      System.out.println("Configure files not found for Swerve Drive!");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
