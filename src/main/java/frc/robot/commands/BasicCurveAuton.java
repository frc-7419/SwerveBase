// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import com.choreo.lib.Choreo;
// import com.choreo.lib.ChoreoTrajectory;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.drive.DriveBaseSubsystem;

// public class BasicCurveAuton extends Command {

//   private final DriveBaseSubsystem driveBaseSubsystem;

//   public BasicCurveAuton(DriveBaseSubsystem driveBaseSubsystem) {
//     this.driveBaseSubsystem = driveBaseSubsystem;
//     addRequirements(driveBaseSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     driveBaseSubsystem.coast();
//     driveBaseSubsystem.setModuleStates(new ChassisSpeeds(0, 0, 0));
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     driveBaseSubsystem.setModuleStates(new ChassisSpeeds(0, 0, 0));
//     driveBaseSubsystem.brake();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }