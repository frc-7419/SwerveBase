// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * This is where the low-level logic for swerve is handled
 */
public class SwerveModule {
    private final CANSparkMax turnMotor;
    private final CANSparkMax driveMotor;
    private final CANCoder turnEncoder;
    private final RelativeEncoder driveEncoder;
    private final PIDController angleController;
    private final String module;

    /**
     * Makes a new swerve module, this handles one turn motor and one drive motor
     * @param turnMotorID is a CAN ID parameter (int)
     * @param driveMotorID is a CAN ID parameter (int)
     * @param turnEncoderID is a CAN ID parameter (int)
     * @param turnEncoderOffset is absolute pos at zero in deg (double)
     * @param module for naming modules during comprehensive shuffleboard outputs (String)
     */
    public SwerveModule(int turnMotorID, int driveMotorID, int turnEncoderID, double turnEncoderOffset, String module) {
        this.module = module;
        // PID init
        angleController = new PIDController(Constants.SwerveConstants.anglekP, Constants.SwerveConstants.anglekI, Constants.SwerveConstants.anglekD);
        angleController.enableContinuousInput(0, 360);
        angleController.setTolerance(0.5);
        // Turn motor init
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        turnMotor.setIdleMode(IdleMode.kCoast);
        // Drive motor init
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor.setIdleMode(IdleMode.kCoast);
        // Turn encoder init
        turnEncoder = new CANCoder(turnEncoderID);
        turnEncoder.configFactoryDefault();
        turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        turnEncoder.configMagnetOffset(turnEncoderOffset);
        turnEncoder.configSensorDirection(false);
        // Drive encoder init
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(1/((1/Constants.SwerveConstants.kWheelCircumfrence)*5.5));
        driveEncoder.setVelocityConversionFactor(1); //TODO: fix this
    }

    public void coast() {
        turnMotor.setIdleMode(IdleMode.kCoast);
        driveMotor.setIdleMode(IdleMode.kCoast);
    }
    
    public void brake() {
        turnMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setIdleMode(IdleMode.kBrake);
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public void resetDriveEncoder() {
        driveEncoder.setPosition(0);
    }

    public boolean reachedDist(double meters) {
        return Math.abs(driveEncoder.getPosition()) > meters;
    }

    public SwerveModulePosition getPose() {
        return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition()));
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition()));
    }

    public void setSwerveModuleState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition()));
        state.speedMetersPerSecond *= state.angle.minus(Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition())).getCos();
        driveMotor.set(state.speedMetersPerSecond);
        turnMotor.set(-MathUtil.clamp(angleController.calculate(turnEncoder.getAbsolutePosition(), state.angle.getDegrees()) , -Constants.SwerveModuleConstants.kMaxTurningSpeed, Constants.SwerveModuleConstants.kMaxTurningSpeed));
    }

    /**
     * Stops all motors in the module
     */
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
    
    /**
     * Outputs values to dashboard
     */
    public void outputDashboard() {
        SmartDashboard.putNumber(module+" angle", turnEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(module+" driveEncoder", getDrivePosition());
    }
}