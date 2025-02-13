// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.SwerveModule;
import frc.robot.Configs.SwerveModuleConfigs;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
      SwerveDrivebaseConstants.kFrontLeftTurnID,
      SwerveDrivebaseConstants.kFrontLeftCANCoderID,
      SwerveDrivebaseConstants.kFrontLeftCANCoderMagnetOffset,
      SwerveDrivebaseConstants.kFrontLeftCANCoderAbsoluteSensorDiscontinuityPoint
    );

    m_frontRight = new SwerveModule(
      SwerveDrivebaseConstants.kFrontRightDriveID,
      SwerveDrivebaseConstants.kFrontRightTurnID,
      SwerveDrivebaseConstants.kFrontRightCANCoderID,
      SwerveDrivebaseConstants.kFrontRightCANCoderMagnetOffset,
      SwerveDrivebaseConstants.kFrontRightCANCoderAbsoluteSensorDiscontinuityPoint
    );

    m_backLeft = new SwerveModule(
      SwerveDrivebaseConstants.kBackLeftDriveID,
      SwerveDrivebaseConstants.kBackLeftTurnID,
      SwerveDrivebaseConstants.kBackLeftCANCoderID,
      SwerveDrivebaseConstants.kBackLeftCANCoderMagnetOffset,
      SwerveDrivebaseConstants.kBackLeftCANCoderAbsoluteSensorDiscontinuityPoint
    );

    m_backRight = new SwerveModule(
      SwerveDrivebaseConstants.kBackRightDriveID,
      SwerveDrivebaseConstants.kBackRightTurnID,
      SwerveDrivebaseConstants.kBackRightCANCoderID,
      SwerveDrivebaseConstants.kBackRightCANCoderMagnetOffset,
      SwerveDrivebaseConstants.kBackRightCANCoderAbsoluteSensorDiscontinuityPoint
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

public Command testMotorsCommand(Supplier<Double> speed, Supplier<Double> steer) {
  return this.run(() -> runMotors(speed.get(), steer.get()));
}

  //for debugging
  private void runMotors(double speed, double steer) {
    m_frontLeft.turnMotors(speed, steer);
    m_frontRight.turnMotors(speed, steer);
    m_backLeft.turnMotors(speed, steer);
    m_backRight.turnMotors(speed, steer);
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public void resetEncoders(){
    m_frontLeft.resetEncoder();
    m_frontRight.resetEncoder();
    m_backLeft.resetEncoder();
    m_backRight.resetEncoder();
  }

  public void configureCANCoders() {
    m_frontLeft.configMagnets(SwerveDrivebaseConstants.kFrontLeftCANCoderMagnetOffset, SwerveDrivebaseConstants.kFrontLeftCANCoderAbsoluteSensorDiscontinuityPoint);
    m_frontRight.configMagnets(SwerveDrivebaseConstants.kFrontRightCANCoderMagnetOffset, SwerveDrivebaseConstants.kFrontRightCANCoderAbsoluteSensorDiscontinuityPoint);
    m_backLeft.configMagnets(SwerveDrivebaseConstants.kBackLeftCANCoderMagnetOffset, SwerveDrivebaseConstants.kBackLeftCANCoderAbsoluteSensorDiscontinuityPoint);
    m_backRight.configMagnets(SwerveDrivebaseConstants.kBackRightCANCoderMagnetOffset, SwerveDrivebaseConstants.kBackRightCANCoderAbsoluteSensorDiscontinuityPoint);
  }
 
  public void periodic() {
    SmartDashboardTunePID();
  }

  public void SmartDashboardTunePID()
  {
    m_frontLeft.logData("FL");
    m_frontRight.logData("FR");
    m_backLeft.logData("BL");
    m_backRight.logData("BR");
    
    double driveP = SmartDashboard.getNumber("driveP", 0);
    double driveI = SmartDashboard.getNumber("driveI", 0);
    double driveD = SmartDashboard.getNumber("driveD", 0);
    double turnP = SmartDashboard.getNumber("turnP", 0);
    double turnI = SmartDashboard.getNumber("turnI", 0);
    double turnD = SmartDashboard.getNumber("turnD", 0);
    
    SmartDashboard.putNumber("front left P",m_frontLeft.getConfigAccessor().closedLoop.getP());
    
    if ((SwerveConstants.driveP != driveP) || 
    (SwerveConstants.driveI != driveI) || 
    (SwerveConstants.driveD != driveD) || 
    (SwerveConstants.turnP != turnP) || 
    (SwerveConstants.turnI != turnI) || 
    (SwerveConstants.turnD != turnD) ){
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