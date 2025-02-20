package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelight.LimelightHelpers;
import frc.robot.subsystems.Swerve.DriveSubsystem;

// Positions Robot at the Nearest Valid Target
public class AimNRangeReefRightCommand extends Command {
    
  // Instantiate Stuff
  DriveSubsystem m_driveSubsystem;

  // Timer for cancellation with failed robot adjustment
  Timer timer = new Timer();

  // PID Controller stuff (woah so many so scary) 
  PIDController mAimController = new PIDController(VisionConstants.kPAim, VisionConstants.kIAim, VisionConstants.kDAim);
  PIDController mRangeController = new PIDController(VisionConstants.kPRange, VisionConstants.kIRange, VisionConstants.kDRange);
  PIDController mStrafeController = new PIDController(VisionConstants.kPStrafe, VisionConstants.kIStrafe, VisionConstants.kDStrafe);
    
  // All the Valid IDs available for positioning
  int[] validIDs = {6, 7, 8, 9, 10, 11};

  // Valid Tag
  int validTag;

  // Bot Pose Target Space Relative (TX, TY, TZ, Pitch, Yaw, Roll)
  private double[] botPoseTargetSpace = new double[6];

  // Lil boolean for checking for "Tag In View" 
  boolean tiv;

  // Constructor
  public AimNRangeReefRightCommand(DriveSubsystem driveSubsystem) {
        
    // Definitions and setting parameters are equal to members!
    m_driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // What we do to set up the command
  public void initialize() {
    
    // Adds condition that filters out undesired IDs
    LimelightHelpers.SetFiducialIDFiltersOverride(VisionConstants.kLimelightName, validIDs);

    // Update BotPoseTargetSpace
    botPoseTargetSpace = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightName).getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    
    
    // Sets up a valid tag
    // validTag = (int) LimelightHelpers.getFiducialID(VisionConstants.kLimelightName);

    // Checks for TIV
    tiv = LimelightHelpers.getTV(VisionConstants.kLimelightName) 
      && botPoseTargetSpace[2] > VisionConstants.kTZValidRange 
      && Math.abs(botPoseTargetSpace[4])< VisionConstants.kYawValidRange;

    // Timer Reset
    timer.start();
    timer.reset();
  }
    
  // The actual control!
  public void execute() {

    // Update the pose from NetworkTables (Limelight Readings)
    botPoseTargetSpace = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightName).getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    if (tiv){
      tiv = LimelightHelpers.getTV(VisionConstants.kLimelightName);
      m_driveSubsystem.drive(limelightRange_PID(), limelightStrafe_PID(), limelightAim_PID(), false);
    }
  }

  // Add stuff we do after to reset here (a.k.a tag filters)
  public void end(boolean interrupted) {}

  // Are we done yet? Finishes when threshold is reached or if no tag in view or if timer is reached 
  public boolean isFinished() {
    return (
      // Range (Distance to Tag)
      Math.abs(botPoseTargetSpace[2]-VisionConstants.kRangeReefRightTarget) < VisionConstants.kRangeReefRightErrorLimit &&
      // Aim (Angle)
      Math.abs(botPoseTargetSpace[4]-VisionConstants.kAimReefRightTarget)  < VisionConstants.kAimReefRightErrorLimit &&
      // Strafe (Right Right Positioning)
      Math.abs(botPoseTargetSpace[0]-VisionConstants.kStrafeReefRightTarget)  < VisionConstants.kStrafeReefRightErrorLimit)
      // Other quit conditions
      || !tiv || timer.get() > 3;

  }

  // Advanced PID-assisted ranging control with Limelight's TZ value from target-relative data
  private double limelightRange_PID() {

    // Limelight Z Axis Range in meters
    mRangeController.enableContinuousInput(-3, 0); 
    
    // Calculates response based on difference in distance from tag to robot
    double targetingForwardSpeed = mRangeController.calculate(botPoseTargetSpace[2] - VisionConstants.kRangeReefRightTarget);

    // Value scale up to robot max speed and invert (double cannot exceed 1.0)
    targetingForwardSpeed *= 1.0 * DriveConstants.kMaxSpeedMetersPerSecond;

    // Hooray
    return targetingForwardSpeed;
  }

  // Advanced PID-assisted ranging control with Limelight's TX value from target-relative data
  private double limelightStrafe_PID() {

    // Limelight X Axis Range in Meters
    mStrafeController.enableContinuousInput(-3, 3);
    
    // Calculates response based on difference in horizontal distance from tag to robot
    double targetingStrafeSpeed = mStrafeController.calculate(botPoseTargetSpace[0] - VisionConstants.kStrafeReefRightTarget);

    // Value scale up to robot max speed (Double can't exceed 1.0)
    targetingStrafeSpeed *= -0.7 * DriveConstants.kMaxSpeedMetersPerSecond;

    // Hooray
    return targetingStrafeSpeed;
  }

  // Advanced PID-assisted ranging control with Limelight's Yaw value from target-relative data
  private double limelightAim_PID() {

    // Limelight Yaw Angle in Meters
    mAimController.enableContinuousInput(-30, 30); 
    
    // Calculates response based on difference in angle from tag to robot
    double targetingAngularVelocity = mAimController.calculate(botPoseTargetSpace[4] - VisionConstants.kAimReefRightTarget);
    
    // Multiply by -1 because robot is CCW Positive. Multiply by a reduction 
    // multiplier to reduce speed. Scale TX up with robot speed.
    targetingAngularVelocity *= -0.1 * DriveConstants.kMaxAngularSpeed;

    // Hooray
    return targetingAngularVelocity;
  }
}
