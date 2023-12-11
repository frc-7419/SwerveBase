package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AdvancedCurveAuton;
import frc.robot.commands.SwerveDriveFieldCentric;
import frc.robot.subsystems.drive.DriveBaseSubsystem;


public class RobotContainer {
  
  // Joysticks, subsystems, and commands must all be private and final

  // Joysticks
  private final XboxController driver = new XboxController(0); //driver

  //Subsystems
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();

  //Commands
  private final SwerveDriveFieldCentric swerveDriveFieldCentric = new SwerveDriveFieldCentric(driver, driveBaseSubsystem);
  private final SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  private final ChoreoTrajectory traj;
  Field2d m_field = new Field2d();

  /**
   * Creates new RobotContainer and configures auton and buttons
   */
  public RobotContainer() {
    //Setting up Traj - needs to be here to prevent delay in auton

    traj = Choreo.getTrajectory("FullHomo");

    m_field.getObject("traj").setPoses(
        traj.getInitialPose(), traj.getFinalPose()
    );
    m_field.getObject("trajPoses").setPoses(
        traj.getPoses()
    );

    driveBaseSubsystem.resetOdometry(traj.getInitialPose());

    configureButtonBindings();
    configureAutoSelector();
  }
  /**
   * This method is for configuring the button bindings on the joysticks
   */
  private void configureButtonBindings() {

  }
  /**
   * This method is for configuring the auton chooser
   */
  private void configureAutoSelector() {
    SmartDashboard.putData("Auton", autonomousChooser);
  }
  /**
   * This will be the method which gets called to 
   * @return Auton command
   */
  public Command getAutonomousCommand() {
      return new AdvancedCurveAuton(driveBaseSubsystem, traj);
  }
  /**
   * Sets default commands to be used for teleop
   */
  public void setDefaultCommands() {
    driveBaseSubsystem.setDefaultCommand(swerveDriveFieldCentric);
  }
}