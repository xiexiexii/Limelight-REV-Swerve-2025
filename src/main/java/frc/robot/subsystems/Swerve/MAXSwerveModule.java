// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;

// MAX Swerve Module Class
public class MAXSwerveModule {
    
  // Motors, Encoders, PID for the driving and turning motors
  private final SparkMax m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  // Ang Offset is the rotation from default position
  private double m_chassisAngularOffset = 0;

  // SMS is the speed and rotation of the Swerve Module
  private SwerveModuleState m_desiredSwerveModuleState = new SwerveModuleState(0.0, new Rotation2d());

  // Constructor for a new MAX Swerve Module yay
  public MAXSwerveModule(int drivingCANID, int turningCANID, double chassisAngularOffset) {

    // Initializes Driving and Turning Motors
    m_drivingSparkMax = new SparkMax(drivingCANID, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANID, MotorType.kBrushless);
    
    // Set up encoders for drive and turn SparkMAX
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();

    // Sets up closed loop controller (PID) for drive and turn SparkMAX
    m_drivingClosedLoopController = m_drivingSparkMax.getClosedLoopController();
    m_turningClosedLoopController = m_turningSparkMax.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSparkMax.configure(
      Configs.MAXSwerveModule.drivingConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    m_turningSparkMax.configure(
      Configs.MAXSwerveModule.turningConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    
    // Takes care of angular offset and resets
    // driving encoder to zero 
    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredSwerveModuleState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  // Returns the current state of the module
  // Subtracts offset to convert field relative to chassis relative
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_drivingEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  // Returns the current position of the module
  // Also does the chassis relative conversion yay
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_drivingEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  // Sets the desired state for the module with speed and angle
  public void setDesiredState(SwerveModuleState desiredState) {
    // Applies angular offset to make it field relative
    SwerveModuleState correctedDesiredState= new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize reference to avoid turning more than 90 degrees
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command the NEOs to move towards their set points
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    // Sets the corrected desired state as the new desired state
    m_desiredSwerveModuleState = desiredState;
  }

  // Zeros all encoders on SwerveModule
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
