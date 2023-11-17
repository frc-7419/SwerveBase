// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBaseSubsystem extends SubsystemBase {
  private final SwerveModule[] swerveModules;
  private SwerveDriveOdometry m_odometry; // TODO: add this
  private SwerveModulePosition[] positions; // TODO: whatever ur suposed to do with this
  private final AHRS ahrs;

  public DriveBaseSubsystem() {
    swerveModules = new SwerveModule[] {
        new SwerveModule(SwerveConstants.frontLeft.turnMotorID, SwerveConstants.frontLeft.driveMotorID,
         SwerveConstants.frontLeft.turnEncoderID, SwerveConstants.frontLeft.absolutePositionAtRobotZero, "FrontLeftModule"),
        new SwerveModule(SwerveConstants.frontRight.turnMotorID, SwerveConstants.frontRight.driveMotorID,
         SwerveConstants.frontRight.turnEncoderID, SwerveConstants.frontRight.absolutePositionAtRobotZero, "FrontRightModule"),
        new SwerveModule(SwerveConstants.backRight.turnMotorID, SwerveConstants.backRight.driveMotorID,
         SwerveConstants.backRight.turnEncoderID, SwerveConstants.backRight.absolutePositionAtRobotZero, "BackRightModule"),
        new SwerveModule(SwerveConstants.backLeft.turnMotorID, SwerveConstants.backLeft.driveMotorID,
         SwerveConstants.backLeft.turnEncoderID, SwerveConstants.backLeft.absolutePositionAtRobotZero, "BackLeftModule"),
    };
    ahrs = new AHRS(SerialPort.Port.kMXP);
    ahrs.zeroYaw(); // field centric, we need yaw to be zero
    coast();
  }

  public void zeroYaw() {
    ahrs.zeroYaw();
  }

  public SwerveModule getSwerveModule(int index) {
    return swerveModules[index];
  }

  public double getYaw() { // CW IS POSITIVE BY DEFAULT
    return -ahrs.getYaw();
  }

  public double getPitch() {
    return ahrs.getPitch();
  }

  public double getRoll() {
    return ahrs.getRoll();
  }

  public boolean reachedDist(double meters) {
    return (swerveModules[0].reachedDist(meters)) &&
           (swerveModules[1].reachedDist(meters)) &&
           (swerveModules[2].reachedDist(meters)) &&
           (swerveModules[3].reachedDist(meters));
  }

  public void resetDriveEnc() {
    for (SwerveModule s : swerveModules) s.resetDriveEnc();
  }

  public Rotation2d getRotation2d() {
    return ahrs.getRotation2d();
    /*
     * the thing is .getYaw is -180 to 180 so it not being 0 to 360
     * may cause the internal conversion that Rotation2d does to be wrong
     */
  }

  public void brake() {
    for (SwerveModule s : swerveModules) s.brake();
  }

  public void coast() {
    for (SwerveModule s : swerveModules) s.coast();
  }

  public void stop() {
    for (SwerveModule s : swerveModules) s.stop();
  }

  /**
   * Returns chassis speeds from field-centric joystick controls. This is what determines the translational speed of the robot in proportion to joystick values.
   * @param joystick
   * @return
   */
  public ChassisSpeeds getChassisSpeedsFromJoystick(double vx, double vy, double rx, boolean slowMode) {
    vx = Math.abs(vx)>0.05?vx*SwerveConstants.maxTranslationalSpeed:0;
    vy = Math.abs(vy)>0.05?vy*SwerveConstants.maxTranslationalSpeed:0;
    rx = Math.abs(rx)>0.05?-0.7*rx*SwerveConstants.maxRotationalSpeed:0;
    if(slowMode) {
      vx *= 0.2;
      vy *= 0.2;
      rx *= 0.2;
    }
    return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rx, getRotation2d());
  }

  /**
   * Sets the individual swerve module states
   * @param moduleStates
   */
  public void setModuleStates(SwerveModuleState[] moduleStates) {
    for (int i=0; i<4; ++i) swerveModules[i].setSwerveModuleState(moduleStates[i]);
  }

  /**
   * Sets the individual swerve module states
   * @param moduleStates
   */
  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] moduleStates = Constants.SwerveConstants.kSwerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    for (int i=0; i<4; ++i) swerveModules[i].setSwerveModuleState(moduleStates[i]);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber( "Yaw", getYaw());
    for(SwerveModule s : swerveModules) s.outputDashboard();
  }
}