package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelight.LimelightHelpers;
import frc.robot.subsystems.Swerve.DriveSubsystem;

// Drives Robot to a set distance from the Nearest Valid Target
public class RangeCommand extends Command {
    
  // Instantiate Stuff
  DriveSubsystem m_driveSubsystem;

  // Timer for cancellation with failed robot adjustment
  Timer timer = new Timer();

  // PID Controller Hooray
  PIDController m_rangeController = new PIDController(VisionConstants.kPRange, VisionConstants.kIRange, VisionConstants.kDRange);
    
  // All the Valid IDs available for positioning
  int[] validIDs = {6, 7, 8, 9, 10, 11};

  // Lil boolean for checking for "Tag In View" 
  boolean tiv;

  // Constructor
  public RangeCommand(DriveSubsystem driveSubsystem) {
        
    // Definitions and setting parameters are equal to members!
    m_driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // What we do to set up the command
  public void initialize() {
    
    // Adds condition that filters out undesired IDs
    LimelightHelpers.SetFiducialIDFiltersOverride(VisionConstants.kLimelightName, validIDs);

    // Checks for TIV
    tiv = LimelightHelpers.getTV(VisionConstants.kLimelightName);

    // Timer Reset
    timer.start();
    timer.reset();
  }
    
  // The actual control!
  public void execute() {

    // If tags are in view, drive forward at a speed proportional to the offset robot relative!
    if (tiv) m_driveSubsystem.drive(limelight_range_proportional(), 0, 0, false);

    // Otherwise we tell it to quit
    else tiv = false;
  }

  // Add stuff we do after to reset here (a.k.a tag filters)
  public void end(boolean interrupted) {}

  // Are we done yet? Finishes when threshold is reached or if no tag in view or if timer is reached
  public boolean isFinished() {
    return (LimelightHelpers.getTY(VisionConstants.kLimelightName) < VisionConstants.kRangeThresholdMax 
      && LimelightHelpers.getTY(VisionConstants.kLimelightName) > VisionConstants.kRangeThresholdMin) 
      || !tiv || timer.get() > 3;
  }

  // Simple ranging control with Limelight's TY value
  // Works best if your Limelight's mount height and target mount height are different.
  // Use TA (area) for target ranging rather than TY if LL and target are on the same level
  private double limelight_range_proportional() {
    m_rangeController.enableContinuousInput(-30, 30);
    
    // Calculates based on difference in distance from tag to robot
    double targetingForwardSpeed = m_rangeController.calculate(LimelightHelpers.getTY(VisionConstants.kLimelightName) - VisionConstants.kRangeTarget);

    // Scale up to robot speed and 30% speed reduction
    targetingForwardSpeed *= 0.7 * DriveConstants.kMaxSpeedMetersPerSecond;

    // Enjoy
    return targetingForwardSpeed;
  }
}
