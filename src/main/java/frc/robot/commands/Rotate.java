// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class Rotate extends Command {
  private final DriveBaseSubsystem driveBaseSubsystem;
  private final double degrees;
  public Rotate(DriveBaseSubsystem driveBaseSubsystem, double degrees) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.degrees = degrees;
    addRequirements(driveBaseSubsystem);
  }

  @Override
  public void initialize() {
    driveBaseSubsystem.coast();
  }

  @Override
  public void execute() {
    driveBaseSubsystem.setModuleStates(new ChassisSpeeds(0,0,Math.PI/6));
  }

  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.setModuleStates(new ChassisSpeeds(0,0,0));
    driveBaseSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(degrees-(driveBaseSubsystem.getYaw()+320.4))<3;
  }
}
