package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SwerveDriveFieldCentric;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class RobotContainer {
  
  ChoreoTrajectory trajectory = Choreo.getTrajectory("NewPath");
  // Joysticks, subsystems, and commands must all be private and final

  // Joysticks
  private final XboxController driver = new XboxController(0); //driver

  //Subsystems
  private final DriveBaseSubsystem driveBase = new DriveBaseSubsystem();

  //Commands
  private final SwerveDriveFieldCentric swerveDriveFieldCentric = new SwerveDriveFieldCentric(driver, driveBase);
  private final SendableChooser<Command> autonomousChooser = new SendableChooser<>();
  /**
   * Creates new RobotContainer and configures auton and buttons
   */
  public RobotContainer() {
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

    // arbitrary for now
    // PIDController angleController = new PIDController(0.05, 0, 0);
    // angleController.enableContinuousInput(-Math.PI, Math.PI);

    

    return driveBase.followPathCommand("testPath");
    // return new TranslateDistance(driveBase, 1, 0);
  }
  /**
   * Sets default commands to be used for teleop
   */
  public void setDefaultCommands() {
    driveBase.setDefaultCommand(swerveDriveFieldCentric);
  }
}