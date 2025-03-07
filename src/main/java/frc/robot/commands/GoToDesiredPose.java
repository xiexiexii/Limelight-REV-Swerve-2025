package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelight.LimelightHelpers;
import frc.robot.subsystems.Swerve.DriveSubsystem;

// Positions Robot at the Nearest Valid Target
public class GoToDesiredPose extends Command {
    
  // Instantiate Stuff
  DriveSubsystem m_driveSubsystem;
  Pose2d targetPose;

  // Timer for cancellation with failed robot adjustment
  Timer timer = new Timer();

  // PID Controller stuff (woah so many so scary) 
  PIDController mThetaController = new PIDController(VisionConstants.kPAim, VisionConstants.kIAim, VisionConstants.kDAim);
  PIDController mXController = new PIDController(VisionConstants.kPRange, VisionConstants.kIRange, VisionConstants.kDRange);
  PIDController mYController = new PIDController(VisionConstants.kPStrafe, VisionConstants.kIStrafe, VisionConstants.kDStrafe);
    
  // All the Valid IDs available for positioning
  int[] validIDs = {6, 7, 8, 9, 10, 11};

  // Valid Tag
  int validTag;

  Pose2d currentPose;

  // Lil boolean for checking for "Tag In View" 
  boolean tiv;

  // Constructor
  public GoToDesiredPose(DriveSubsystem driveSubsystem, Pose2d targetPose) {
        
    // Definitions and setting parameters are equal to members!
    m_driveSubsystem = driveSubsystem;
    this.targetPose = targetPose;
    addRequirements(driveSubsystem);
  }

  // What we do to set up the command
  public void initialize() {
    // Timer Reset
    timer.start();
    timer.reset();
    currentPose = m_driveSubsystem.getPose();
  }
    
  // The actual control!
  public void execute() {
    currentPose = m_driveSubsystem.getPose();

    m_driveSubsystem.drive(limelightRange_PID(), limelightStrafe_PID(), limelightAim_PID(), true);
  }

  // Add stuff we do after to reset here (a.k.a tag filters)
  public void end(boolean interrupted) {}

  // Are we done yet? Finishes when threshold is reached or if no tag in view or if timer is reached 
  public boolean isFinished() {
    return (
      Math.abs(currentPose.getX()-targetPose.getX()) < VisionConstants.kXErrorLimit &&

      Math.abs(currentPose.getY()-targetPose.getY())  < VisionConstants.kYErrorLimit &&

      Math.abs(currentPose.getRotation().getDegrees()-targetPose.getRotation().getDegrees())  < VisionConstants.kThetaErrorLimit)
      // Other quit conditions
      || timer.get() > 3;

  }

  // Advanced PID-assisted ranging control with Limelight's TZ value from target-relative data
  private double limelightRange_PID() {

    // Limelight Z Axis Range in meters
    mYController.enableContinuousInput(targetPose.getY()-2, targetPose.getY()+2); 
    
    // Calculates response based on difference in distance from tag to robot
    double targetingForwardSpeed = mYController.calculate(currentPose.getY() - targetPose.getY());

    // Value scale up to robot max speed and invert (double cannot exceed 1.0)
    targetingForwardSpeed *= 0.5 * DriveConstants.kMaxSpeedMetersPerSecond;

    // Hooray
    return targetingForwardSpeed;
  }

  // Advanced PID-assisted ranging control with Limelight's TX value from target-relative data
  private double limelightStrafe_PID() {

    // Limelight X Axis Range in Meters
    mXController.enableContinuousInput(targetPose.getX()-2,targetPose.getX()+2);
    
    // Calculates response based on difference in horizontal distance from tag to robot
    double targetingStrafeSpeed = mXController.calculate(currentPose.getX() - targetPose.getX());

    // Value scale up to robot max speed (Double can't exceed 1.0)
    targetingStrafeSpeed *= -0.7 * DriveConstants.kMaxSpeedMetersPerSecond;

    // Hooray
    return targetingStrafeSpeed;
  }

  // Advanced PID-assisted ranging control with Limelight's Yaw value from target-relative data
  private double limelightAim_PID() {

    // Limelight Yaw Angle in Meters
    mThetaController.enableContinuousInput(-180, 180); 
    
    // Calculates response based on difference in angle from tag to robot
    double targetingAngularVelocity = mThetaController.calculate((currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees()+540)%360-180);
    
    // Multiply by -1 because robot is CCW Positive. Multiply by a reduction 
    // multiplier to reduce speed. Scale TX up with robot speed.
    targetingAngularVelocity *= -0.1 * DriveConstants.kMaxAngularSpeed;

    // Hooray
    return targetingAngularVelocity;
  }
}
