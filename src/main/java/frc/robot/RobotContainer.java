// Imports stuff (again!)

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.LocalizationConstants;
import frc.robot.commands.GoToDesiredPose;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

// This class is where the bulk of the robot should be declared.  Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the Robot
// periodic methods (other than the scheduler calls).  Instead, the structure of the robot
// (including subsystems, commands, and button mappings) should be declared here.

public class RobotContainer {

  // Robot's Subsystems
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Controllers
  CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.kDriverControllerPort);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Configure Default Commands
    m_robotDrive.setDefaultCommand(
      
      // Translation of the robot is controlled by the left stick
      // Turning is controlled by the X axis of the right stick
      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), ControllerConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX(), ControllerConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX(), ControllerConstants.kDriveDeadband),
          true),
        m_robotDrive
      )
    );

    // Chooser window on SmartDashboard/Shuffleboard/Elastic
    SmartDashboard.putData("AutoMode", m_chooser);

    // Autos
    m_chooser.addOption("Curvy yay", m_robotDrive.getAuto("Curvy yay"));
  }

  // Define Button and Axis bindings here
  private void configureButtonBindings() {

    // Zero Gyro - Start Button
    // new JoystickButton(m_driverController.getHID(), ControllerConstants.kStart)
    // .onTrue(
    //   new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)
    // );

    // // Sets wheels in an X position to prevent movement - A
    // new JoystickButton(m_driverController.getHID(), ControllerConstants.kA)
    //   .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive)
    // );
    // // Aim Command - B
    // new POVButton(m_driverController.getHID(), ControllerConstants.kB)
    // .onTrue(
    //   new AimCommand(m_robotDrive)
    // );
    // Goes to Red Reef KL - Y
    new JoystickButton(m_driverController.getHID(), ControllerConstants.kY)
    .onTrue(
      new InstantCommand(() -> m_robotDrive.goToDesiredPose(LocalizationConstants.kRedReefKL), m_robotDrive)
    );
    // Go to (16,4,0) - B
    new JoystickButton(m_driverController.getHID(), ControllerConstants.kB)
    .onTrue(
      new GoToDesiredPose(m_robotDrive,new Pose2d(16,4,new Rotation2d(0)))
    );

    // Just Aim - Y
    // new POVButton(m_driverController.getHID(), ControllerConstants.kY)
    // .onTrue(
    //   new AimCommand(m_robotDrive)
    // );

    // // AimNRange Reef Right - D-Pad Right
    // new POVButton(m_driverController.getHID(), ControllerConstants.kDpadRight)
    // .onTrue(
    //   new AimNRangeReefRightCommand(m_robotDrive)
    // );

    // // AimNRange Reef Left - D-Pad Left
    // new POVButton(m_driverController.getHID(), ControllerConstants.kDpadLeft)
    // .onTrue(
    //   new AimNRangeReefLeftCommand(m_robotDrive)
    // );
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();   
  } 
}