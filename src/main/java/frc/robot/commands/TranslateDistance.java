// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class TranslateDistance extends CommandBase {
  private final DriveBaseSubsystem driveBaseSubsystem;
  private final double xVelocity;
  private final double yVelocity;
  private final double totalDistance;
  private final ChassisSpeeds chassisSpeeds;

  public TranslateDistance(DriveBaseSubsystem driveBaseSubsystem, double xDistance, double yDistance) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.xVelocity = xDistance==0?0:0.2*(xDistance/Math.abs(xDistance));
    this.yVelocity = yDistance==0?0:0.2*(yDistance/Math.abs(yDistance));
    this.totalDistance = Math.sqrt((xDistance*xDistance)+(yDistance*yDistance));
    this.chassisSpeeds = new ChassisSpeeds(xVelocity, yVelocity,0);
    addRequirements(driveBaseSubsystem);
  }

  
  @Override
  public void initialize() {
    driveBaseSubsystem.resetDriveEnc();
    
  }

  @Override
  public void execute() {
    driveBaseSubsystem.setModuleStates(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.setModuleStates(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveBaseSubsystem.reachedDist(totalDistance);
  }
}
