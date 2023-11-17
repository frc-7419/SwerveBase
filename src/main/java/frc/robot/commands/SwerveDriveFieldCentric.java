// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class SwerveDriveFieldCentric extends CommandBase {
  private XboxController joystick;
  private DriveBaseSubsystem driveBaseSubsystem;

  public SwerveDriveFieldCentric(XboxController joystick, DriveBaseSubsystem driveBaseSubsystem) {
    this.joystick = joystick;
    this.driveBaseSubsystem = driveBaseSubsystem;
    addRequirements(driveBaseSubsystem);
  }

  @Override
  public void initialize() {
    driveBaseSubsystem.coast();
    driveBaseSubsystem.zeroYaw();
  }

  @Override
  public void execute() {
    driveBaseSubsystem.setModuleStatesFromJoystick(joystick);
  }

  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.brake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}