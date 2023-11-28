// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
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
  private final SwerveDriveOdometry m_odometry;
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;
  private final AHRS ahrs;

  public DriveBaseSubsystem() {
    frontLeftModule = new SwerveModule(SwerveConstants.frontLeft.turnMotorID, SwerveConstants.frontLeft.driveMotorID, SwerveConstants.frontLeft.turnEncoderID, SwerveConstants.frontLeft.offset, "FrontLeftModule");
    frontRightModule = new SwerveModule(SwerveConstants.frontRight.turnMotorID, SwerveConstants.frontRight.driveMotorID, SwerveConstants.frontRight.turnEncoderID, SwerveConstants.frontRight.offset, "FrontRightModule");
    backLeftModule = new SwerveModule(SwerveConstants.backLeft.turnMotorID, SwerveConstants.backLeft.driveMotorID, SwerveConstants.backLeft.turnEncoderID, SwerveConstants.backLeft.offset, "BackLeftModule");
    backRightModule = new SwerveModule(SwerveConstants.backRight.turnMotorID, SwerveConstants.backRight.driveMotorID, SwerveConstants.backRight.turnEncoderID, SwerveConstants.backRight.offset, "BackRightModule");
    ahrs = new AHRS(SerialPort.Port.kMXP);
    ahrs.zeroYaw(); // field centric, we need yaw to be zero
    m_odometry = new SwerveDriveOdometry(Constants.SwerveConstants.m_SwerveDriveKinematics, ahrs.getRotation2d(), getPositions());
    coast();
  }



  public void zeroYaw() {
    ahrs.zeroYaw();
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
    return 
    (frontLeftModule.reachedDist(meters))&&
    (frontRightModule.reachedDist(meters))&&
    (backLeftModule.reachedDist(meters))&&
    (backRightModule.reachedDist(meters));
  }

  public void resetDriveEnc() {
    frontLeftModule.resetDriveEncoder();
    frontRightModule.resetDriveEncoder();
    backLeftModule.resetDriveEncoder();
    backRightModule.resetDriveEncoder();
  }

  public Rotation2d getRotation2d() {
    return ahrs.getRotation2d();
    /*
     * the thing is .getYaw is -180 to 180 so it not being 0 to 360
     * may cause the internal conversion that Rotation2d does to be wrong
     */
  }

  public void brake() {
    frontLeftModule.brake();
    frontRightModule.brake();
    backLeftModule.brake();
    backRightModule.brake();
  }

  public void coast() {
    frontLeftModule.coast();
    frontRightModule.coast();
    backLeftModule.coast();
    backRightModule.coast();
  }

  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[]{frontLeftModule.getPose(), frontRightModule.getPose(), backLeftModule.getPose(), backRightModule.getPose()};
  }

  public void stop() {
    frontLeftModule.stop();
    frontRightModule.stop();
    backLeftModule.stop();
    backRightModule.stop();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(ahrs.getRotation2d(), getPositions(), pose);
  }

  /**
   * Returns chassis speeds from field-centric joystick controls. This is what determines the translational speed of the robot in proportion to joystick values.
   * @param joystick
   * @return
   */
  public ChassisSpeeds getChassisSpeedsFromJoystick(double vx, double vy, double rx, boolean slowMode) {
    vx = Math.abs(vx)>0.05?-vx*SwerveConstants.kMaxTranslationalSpeed:0;
    vy = Math.abs(vy)>0.05?vy*SwerveConstants.kMaxTranslationalSpeed:0;
    rx = Math.abs(rx)>0.05?-0.7*rx*SwerveConstants.kMaxRotationalSpeed:0;
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
    frontLeftModule.setSwerveModuleState(moduleStates[0]);
    frontRightModule.setSwerveModuleState(moduleStates[1]);
    backLeftModule.setSwerveModuleState(moduleStates[2]);
    backRightModule.setSwerveModuleState(moduleStates[3]);
  }

  /**
   * Sets the individual swerve module states from chassis speed
   * @param moduleStates
   */
  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    setModuleStates(Constants.SwerveConstants.m_SwerveDriveKinematics.toSwerveModuleStates(chassisSpeeds));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber( "Yaw", getYaw());
    frontLeftModule.outputDashboard();
    frontRightModule.outputDashboard();
    backLeftModule.outputDashboard();
    backRightModule.outputDashboard();
    m_odometry.update(getRotation2d(), getPositions());
  }
}