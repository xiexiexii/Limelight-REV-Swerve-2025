// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

public final class Constants {

  // Drive Subsystem stuff
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAngularSpeed = 1.5 * Math.PI; // radians per second

    // Chassis configuration
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
      
    // Locations of each swerve module
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // DRIVE CAN IDs
    public static final int kFrontLeftDrivingCanId = 6;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 8;
    public static final int kRearRightDrivingCanId = 2;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 3;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 1;

    // Gyro inverts
    public static final boolean kGyroReversed = true;
  }

  // Module Constants for MAXSwerve Modules
  public static final class ModuleConstants {

    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
      / kDrivingMotorReduction;
  }

  // The one random constant that tells you how fast a NEO goes
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  // Controller Ports, Deadband, Buttons and Triggers
  public static final class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;

    public static final int kStart = XboxController.Button.kStart.value;

    public static final int kA = XboxController.Button.kA.value;
    public static final int kB = XboxController.Button.kB.value;
    public static final int kX = XboxController.Button.kX.value;
    public static final int kY = XboxController.Button.kY.value;

    public static final int kDpadRight = 90; // D-Pad Right
    public static final int kDpadLeft = 270; // D-Pad Left
  }

  // Auto stuff
  public static final class AutoConstants {

    // For PathFinding
    public static final PathConstraints kconstraints = new PathConstraints(
      3.0, 
      4.0,
      Units.degreesToRadians(540), 
      Units.degreesToRadians(720)
    );
  }

  // LED Stuff
  public static final class LEDConstants {
    public static final int kBlinkinPort = 0;  
  }

  public static final class VisionConstants {
    // Name
    public static final String kLimelightName = "limelight-three";

    // PID for Tag Relative Control in General
    public static final double kPAim = 0.04;
    public static final double kIAim = 0.000;
    public static final double kDAim = 0.000;

    public static final double kPRange = 0.13;
    public static final double kIRange = 0.0;
    public static final double kDRange = 0.0;

    public static final double kPStrafe = 0.13;
    public static final double kIStrafe = 0.0;
    public static final double kDStrafe = 0.0;

    // Aim/Range
    public static final double kAimThreshold = 0.5;
    public static final double kRangeThresholdMax = -3.8;
    public static final double kRangeThresholdMin = -4.2;
    public static final double kRangeTarget = -4;

    public static final double kTZValidRange = -1.5;
    public static final double kYawValidRange = 35;

    // AimNRange Reef Right
    public static final double kAimReefRightThresholdMax = 0.5;
    public static final double kAimReefRightThresholdMin = -0.5;
    public static final double kAimReefRightTarget = 0;

    public static final double kRangeReefRightThresholdMax = -0.63;
    public static final double kRangeReefRightThresholdMin = -0.68;
    public static final double kRangeReefRightTarget = -0.65;

    public static final double kStrafeReefRightThresholdMax = 0.20;
    public static final double kStrafeReefRightThresholdMin = 0.15;
    public static final double kStrafeReefRightTarget = 0.18;

    // AimNRange Reef Left
    public static final double kAimReefLeftThresholdMax = 0.5;
    public static final double kAimReefLeftThresholdMin = -0.5;
    public static final double kAimReefLeftTarget = 0;

    public static final double kRangeReefLeftThresholdMax = -0.63;
    public static final double kRangeReefLeftThresholdMin = -0.68;
    public static final double kRangeReefLeftTarget = -0.65;

    public static final double kStrafeReefLeftThresholdMax = -0.15;
    public static final double kStrafeReefLeftThresholdMin = -0.20;
    public static final double kStrafeReefLeftTarget = -0.18;

    // Tag Reject Distance
    public static final int kRejectionDistance = 3;

    // Tag Reject Rotation Rate
    public static final int kRejectionRotationRate = 720;
  }

  public static final class LocalizationConstants {
    public static final Pose2d kRedReefKL = new Pose2d(13.97, 1.97, Rotation2d.fromDegrees(112));  
  }
}