// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.SwerveModule;
import frc.robot.Configs.SwerveModuleConfigs;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.SwerveDriveBrake;

import com.studica.frc.AHRS;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveDrivebaseConstants;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;


public class Swerve extends SubsystemBase {
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final SwerveDriveKinematics m_kinematics;

  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  
  public Swerve() {
    m_frontLeft = new SwerveModule(
      SwerveDrivebaseConstants.kFrontLeftDriveID,
      SwerveDrivebaseConstants.kFrontLeftTurnID
    );

    m_frontRight = new SwerveModule(
      SwerveDrivebaseConstants.kFrontRightDriveID,
      SwerveDrivebaseConstants.kFrontRightTurnID
    );

    m_backLeft = new SwerveModule(
      SwerveDrivebaseConstants.kBackLeftDriveID,
      SwerveDrivebaseConstants.kBackLeftTurnID
    );

    m_backRight = new SwerveModule(
      SwerveDrivebaseConstants.kBackRightDriveID,
      SwerveDrivebaseConstants.kBackRightTurnID
    );

    m_kinematics = new SwerveDriveKinematics(
      new Translation2d(SwerveDrivebaseConstants.kWheelBase / 2, SwerveDrivebaseConstants.kTrackWidth / 2),
      new Translation2d(SwerveDrivebaseConstants.kWheelBase / 2, -SwerveDrivebaseConstants.kTrackWidth / 2),
      new Translation2d(-SwerveDrivebaseConstants.kWheelBase / 2, SwerveDrivebaseConstants.kTrackWidth / 2),
      new Translation2d(-SwerveDrivebaseConstants.kWheelBase / 2, -SwerveDrivebaseConstants.kTrackWidth / 2)
    );

    putSmartDashboard();
  }

  public void putSmartDashboard(){
    
    SmartDashboard.putNumber("driveP", 0);
    SmartDashboard.putNumber("driveI", 0);
    SmartDashboard.putNumber("driveD", 0);

    SmartDashboard.putNumber("turnP", 0);
    SmartDashboard.putNumber("turnI", 0);
    SmartDashboard.putNumber("turnD", 0);
  }

 public void drive(double vx, double vy, double omega, boolean fieldRelative) {

    SwerveModuleState[] m_swerveModuleStates;
    if(fieldRelative) {
      m_swerveModuleStates = m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Rotation2d.fromDegrees(-m_gyro.getAngle())));
    } else {
      m_swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(vx, vy, omega));
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(
        m_swerveModuleStates, SwerveDrivebaseConstants.kMaxMetersPerSecond);
    m_frontLeft.setDesiredState(m_swerveModuleStates[0]);
    m_frontRight.setDesiredState(m_swerveModuleStates[1]);
    m_backLeft.setDesiredState(m_swerveModuleStates[2]);
    m_backRight.setDesiredState(m_swerveModuleStates[3]);
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    SmartDashboardTunePID();
  }

  public void SmartDashboardTunePID()
  {
    double driveP = SmartDashboard.getNumber("driveP", 0);
    double driveI = SmartDashboard.getNumber("driveI", 0);
    double driveD = SmartDashboard.getNumber("driveD", 0);

    double turnP = SmartDashboard.getNumber("turnP", 0);
    double turnI = SmartDashboard.getNumber("turnI", 0);
    double turnD = SmartDashboard.getNumber("turnD", 0);
    if ((SwerveConstants.driveP != driveP) || (SwerveConstants.driveI != driveI) || (SwerveConstants.driveD != driveD) || (SwerveConstants.turnP != turnP) || (SwerveConstants.turnI != turnI) || (SwerveConstants.turnD != turnD) ){
      SwerveConstants.driveP = driveP;
      SwerveConstants.driveI = driveI;
      SwerveConstants.driveD = driveD;
      SwerveConstants.turnP = turnP;
      SwerveConstants.turnI = turnI;
      SwerveConstants.turnD = turnD;
      SwerveModuleConfigs.m_configDrive.closedLoop.pid(driveP, driveI, driveD);
      SwerveModuleConfigs.m_configTurn.closedLoop.pid(turnP, turnI, turnD);
      m_frontLeft.configure(SwerveModuleConfigs.m_configDrive, SwerveModuleConfigs.m_configTurn);
      m_frontRight.configure(SwerveModuleConfigs.m_configDrive, SwerveModuleConfigs.m_configTurn);
      m_backLeft.configure(SwerveModuleConfigs.m_configDrive, SwerveModuleConfigs.m_configTurn);
      m_backRight.configure(SwerveModuleConfigs.m_configDrive, SwerveModuleConfigs.m_configTurn);
        
    }
  }
}
