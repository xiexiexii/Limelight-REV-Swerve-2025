// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelight.LimelightHelpers;
import frc.robot.subsystems.Limelight.Localization;
//liam reddy was here
// Drive Subsystem Class yay
public class DriveSubsystem extends SubsystemBase {

  // Creates MAX Swerve Modules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
    DriveConstants.kFrontLeftDrivingCanId,
    DriveConstants.kFrontLeftTurningCanId,
    DriveConstants.kFrontLeftChassisAngularOffset
  );

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
    DriveConstants.kFrontRightDrivingCanId,
    DriveConstants.kFrontRightTurningCanId,
    DriveConstants.kFrontRightChassisAngularOffset
  );

  private final MAXSwerveModule m_backLeft = new MAXSwerveModule(
    DriveConstants.kRearLeftDrivingCanId,
    DriveConstants.kRearLeftTurningCanId,
    DriveConstants.kBackLeftChassisAngularOffset
  );
  
  private final MAXSwerveModule m_backRight = new MAXSwerveModule(
    DriveConstants.kRearRightDrivingCanId,
    DriveConstants.kRearRightTurningCanId,
    DriveConstants.kBackRightChassisAngularOffset
  );

  // Array of modules for Pathplanner
  private MAXSwerveModule[] modules = new MAXSwerveModule[]{
    m_frontLeft,
    m_frontRight,
    m_backLeft,
    m_backRight
  };
  
  // Creates new PP Holonomic Controller
  private PPHolonomicDriveController controller = new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
          new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
          new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        );

  // Creates the Gyro for Swerve Magic
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  // Default rotation for reference (PathPlanner)
  private Rotation2d rawGyroRotation = new Rotation2d();

  // Field2D for Telemetry Data (Limelight)
  private final Field2d m_field = new Field2d();

  // Red Alliance sees forward as 180 degrees, Blue Alliance sees as 0 (Limelight)
  public static int AllianceYaw;

  // Swerve PoseEstimator - Tracks robot pose
  private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics, 
    Rotation2d.fromDegrees(-m_gyro.getYaw()), 
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    },
    new Pose2d()
  );
  
  // Robot Config for PathPlanner
  RobotConfig config;

  // Creates a new DriveSubsystem
  public DriveSubsystem() {

    // Usage reporting for MAXSwerve template
    // HAL = Hardware Abstraction Layer
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    // Get robot config from GUI
    try {
      config = RobotConfig.fromGUISettings();
    } 
    catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::setPose, // Method to reset poseEstimator (will be called if your auto has a starting pose)
      () -> DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::runVelocity, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        controller,
      config, // The robot configuration
      () -> {

        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      // Reference to this subsystem to set requirements
      this 
    );
  }

  // This method will be called once per scheduler run
  // Periodically update the poseEstimator
  public void periodic() {
    m_poseEstimator.update(
      Rotation2d.fromDegrees(-m_gyro.getYaw()), 
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      }
    );
    
    // Updates using LL
    updateVisionMeasurements();

      // Puts Yaw + Angle on Smart Dashboard
      // SmartDashboard.putNumber("NavX Yaw", -m_gyro.getYaw());
      // SmartDashboard.putNumber("NavX Angle", m_gyro.getAngle());
      // SmartDashboard.putNumberArray("Bot Pose Target Space", NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightName).getEntry("botpose_targetspace").getDoubleArray(new double[6]));
      // SmartDashboard.putNumber("Bot Pose 4", NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightName).getEntry("botpose_targetspace").getDoubleArray(new double[6])[4]);
      // SmartDashboard.putBoolean(VisionConstants.kLimelightName + "-tag-in-vision", LimelightHelpers.getTV(VisionConstants.kLimelightName));
  }

  // Updates poseEstimate with the Limelight Readings using MT2 
  public void updateVisionMeasurements() {

    // Setting Yaw to Compensate for Red Alliance Limelight Localization
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        AllianceYaw = 180;
      }
      else if (alliance.get() == DriverStation.Alliance.Blue){
        AllianceYaw = 0;
      }
    }

    // For each limelight...
    for (Localization.LimelightPoseEstimateWrapper estimateWrapper : Localization.getPoseEstimates(getHeading())) {

      // If there is a tag in view and the pose estimate is valid...
      if (estimateWrapper.tiv && poseEstimateIsValid(estimateWrapper.poseEstimate)) {

        // Add the vision measurement to the swerve drive
        m_poseEstimator.addVisionMeasurement(estimateWrapper.poseEstimate.pose,
          estimateWrapper.poseEstimate.timestampSeconds,
          estimateWrapper.getStdvs(estimateWrapper.poseEstimate.avgTagDist));

        // Update position on Field2d
        // m_field.setRobotPose(estimateWrapper.poseEstimate.pose);
        // SmartDashboard.putNumber("local x",estimateWrapper.poseEstimate.pose.getX());
        // SmartDashboard.putNumber("local y",estimateWrapper.poseEstimate.pose.getY());
      }
    }
    
    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
    SmartDashboard.putNumber("local x", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("local y", m_poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Direction Of Travel", directionOfTravel().getDegrees());
  }

  // Check if pose estimate is valid
  private boolean poseEstimateIsValid(LimelightHelpers.PoseEstimate estimate) {
    return estimate != null && Math.abs(getTurnRate()) < VisionConstants.kRejectionRotationRate
      && estimate.avgTagDist < VisionConstants.kRejectionDistance;
  }

  // TODO: [NOT CONFIRMED] Pathfinds to pose and avoids obstacles in the way
  public void pathfindToPose(Pose2d targetPose, PathConstraints constraints) {
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
      targetPose, 
      constraints, 
      0.0
    );
    pathfindingCommand.schedule();
  } 

   // TODO: [NOT CONFIRMED] Pathfinds to path and then follows that path
   public void pathfindThenFollowPath(String path, PathConstraints constraints) {
    try {
      PathPlannerPath m_path = PathPlannerPath.fromPathFile(path);
      Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
      m_path, 
      constraints
    );
    pathfindingCommand.schedule();
    }
    catch(Exception e) {
      e.printStackTrace();
    }
  } 

  // Drives to pose in the shortest way possible without considering obstacles
  public void goToDesiredPose(Pose2d desiredPose){

    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
    // CHECK
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(getPose().getTranslation(), directionOfTravel()),
      desiredPose
    );

    // Create the path using the waypoints created above
    PathPlannerPath path = new PathPlannerPath(
      waypoints,
      AutoConstants.kconstraints,
      null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
      new GoalEndState(0, desiredPose.getRotation()) 
    );
    Command driveTo = new frc.robot.commands.PathfindingCommand(
      path, 
      AutoConstants.kconstraints,
      () -> getPose(), 
      () -> DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()), 
      (ChassisSpeeds speeds) -> runVelocity(speeds), 
      controller, 
      config,
      0.0,
      () -> true,
      () -> false,
      5,
      this);
    driveTo.schedule();
  }
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  // Sets pose for robot (PathPlanner)
  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  // Resets poseEstimator to specified pose
  public void resetPoseEstimator(Pose2d pose) {
    m_poseEstimator.resetPosition(
      Rotation2d.fromDegrees(-m_gyro.getYaw()), 
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      }, 
      pose
    );
  }

   // Method to drive the robot using joystick info
   // xSpeed         Speed of the robot in the x direction (forward).
   // ySpeed         Speed of the robot in the y direction (sideways).
   // rot            Angular rate of the robot.
   // fieldRelative  Whether the provided x and y speeds are relative to the field.
   public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    // Convert from field relative commands to states each module can follow
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeedDelivered, 
        ySpeedDelivered, 
        rotDelivered,
        Rotation2d.fromDegrees(-m_gyro.getYaw())
      ) : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    
    // Ensures wheel speed are physically attainable
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // Set Desired States for each module
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  // Sets wheels to X Formation to keep robot from moving
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  // Sets the Swerve Module States
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  // Gets the Swerve Module States (PathPlanner)
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] currentStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      currentStates[i] = modules[i].getState();
    }
    return currentStates;
  }

  // Gets position of each module (PathPlanner)
  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i <4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  // Robot relative output (PathPlanner)
  public void runVelocity(ChassisSpeeds speeds) {

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      modules[i].setDesiredState(setpointStates[i]);
    }
  }
  
  //
  public Rotation2d directionOfTravel() {
    ChassisSpeeds chassisSpeed = DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    Rotation2d gyro = getPose().getRotation();
    return new Rotation2d(Math.atan2(chassisSpeed.vxMetersPerSecond * gyro.getCos() - chassisSpeed.vyMetersPerSecond * gyro.getSin(),
      chassisSpeed.vyMetersPerSecond * gyro.getCos() + chassisSpeed.vxMetersPerSecond * gyro.getSin()));
}

  // Resets all drive encoders to read position zero
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backLeft.resetEncoders();
    m_backRight.resetEncoders();
  }

  // Zeros the robot heading
  public void zeroHeading() {
    m_gyro.reset();
  }

  // Return Robot Headings, from -180 to 180 degrees
  public double getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getYaw()).getDegrees();
  }


  // Returns Robot Turn Rate, in degrees per second
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  // Gets an Auto from PathPlanner
  public Command getAuto(String autoName) {
    return AutoBuilder.buildAuto(autoName);
  }

}
