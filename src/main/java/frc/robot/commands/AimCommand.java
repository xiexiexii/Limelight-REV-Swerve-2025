package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelight.LimelightHelpers;
import frc.robot.subsystems.Swerve.DriveSubsystem;

// Aims Robot at the Nearest Valid Target
public class AimCommand extends Command {
    
  // Instantiate Stuff
  DriveSubsystem m_driveSubsystem;

  // Timer for cancellation with failed robot adjustment
  Timer timer = new Timer();

  // PID Stuff!
  PIDController m_aimController = new PIDController(VisionConstants.kPAim, VisionConstants.kIAim, VisionConstants.kDAim);
    
  // All the Valid IDs available for positioning
  int[] validIDs = {6, 7, 8, 9, 10, 11};

  // Lil boolean for checking for "Tag In View" 
  boolean tiv;

  // Constructor
  public AimCommand(DriveSubsystem driveSubsystem) {
        
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

    // If tags are in view, rotate at a speed proportional to the offset robot relative!
    if (tiv) m_driveSubsystem.drive(0, 0, limelight_aim_proportional(), false);

    // Otherwise we tell it to quit
    else tiv = false;
  }

  // Add stuff we do after to reset here (a.k.a tag filters)
  public void end(boolean interrupted) {}

  // Are we done yet? Finishes when threshold is reached or if no tag in view or if timer is reached
  public boolean isFinished() {
    return (LimelightHelpers.getTX(VisionConstants.kLimelightName) < VisionConstants.kAimThreshold 
      && LimelightHelpers.getTX(VisionConstants.kLimelightName) > -VisionConstants.kAimThreshold) 
      || !tiv || timer.get() > 2;
  }

  // Method that returns a double for how fast the robot needs to turn, farther angle from the tag is a faster turn
  private double limelight_aim_proportional() {
    m_aimController.enableContinuousInput(-40, 40);
    
    // Proportional multiplier on the X-Offset value
    double targetingAngularVelocity = m_aimController.calculate((LimelightHelpers.getTX(VisionConstants.kLimelightName)));

    // Multiply by -1 because robot is CCW Positive. Multiply by a reduction 
    // multiplier to reduce speed. Scale TX up with robot speed.
    targetingAngularVelocity *= 0.1 * DriveConstants.kMaxAngularSpeed;

    // Hooray
    return targetingAngularVelocity;
  }
}
