// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.choreo.lib.Choreo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AdvancedCurveAuton extends SequentialCommandGroup {
  /** Creates a new AdvancedCurveAuton. */
  public AdvancedCurveAuton(DriveBaseSubsystem driveBaseSubsystem) {
    addRequirements(driveBaseSubsystem);
    addCommands(
      Choreo.choreoSwerveCommand(
      Choreo.getTrajectory("BasicCurve"), 
      driveBaseSubsystem::getPose, 
      new PIDController(Constants.PathPlannerConstants.kPXController, 0.0, 0.0),
      new PIDController(Constants.PathPlannerConstants.kPYController, 0.0, 0.0),
      new PIDController(Constants.PathPlannerConstants.kPThetaController, 0.0, 0.0),
      (ChassisSpeeds speeds) -> 
          driveBaseSubsystem.setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, driveBaseSubsystem.getRotation2d())),
      true,
      driveBaseSubsystem 
    )
    );
  }
}
