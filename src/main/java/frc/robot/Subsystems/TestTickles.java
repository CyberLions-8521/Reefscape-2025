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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
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


public class TestTickles extends SubsystemBase {
  private final SparkMax m_turnMotor = new SparkMax(2, SparkLowLevel.MotorType.kBrushless); //motor 1 
  private final SparkMax m_driveMotor = new SparkMax(50, SparkLowLevel.MotorType.kBrushless); //motor 2 
  private final RelativeEncoder m_driveEncoder = m_driveMotor.getEncoder();
  private final RelativeEncoder m_turnEncoder = m_turnMotor.getEncoder();
  private AHRS m_gyro;

  
  public TestTickles() {
    m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  }


  public void resetGyro() {

  }

  public void resetEncoders(){
    m_driveEncoder.setPosition(0);
    m_turnEncoder.setPosition(0);
  }
 
  public void periodic() {
    // SmartDashboardTunePID();
  }

  public void setSpeed1(){
    m_turnMotor.set(0.25);
  }

  public void setSpeed2(){
    m_driveMotor.set(0.25);
  }

  public void stopMotors(){
    m_turnMotor.set(0);
    m_driveMotor.set(0);
  }

  public void SmartDashboardTunePID()
  {
      SmartDashboard.putNumber(m_turnMotor + "turn position", m_turnEncoder.getPosition());
      SmartDashboard.putNumber(m_turnMotor +  "turn velocity", m_turnEncoder.getVelocity());
      SmartDashboard.putNumber(m_driveMotor + "drive position", m_driveEncoder.getPosition());
      SmartDashboard.putNumber(m_driveMotor +  "drive velocity", m_driveEncoder.getVelocity());
      SmartDashboard.putNumber("Drive Conversion Factor", m_driveMotor.configAccessor.encoder.getPositionConversionFactor());
      SmartDashboard.putNumber("Turn Conversion Factor", m_turnMotor.configAccessor.encoder.getPositionConversionFactor());

  }
}