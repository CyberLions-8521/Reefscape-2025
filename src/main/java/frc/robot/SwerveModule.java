// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.CANCommonJNI;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Configs.SwerveModuleConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

/** Add your docs here. */
public class SwerveModule {
    private SparkMax m_driveMotor; 
    private SparkMax m_turnMotor; 

    private SparkClosedLoopController m_drivePID;
    private SparkClosedLoopController m_turnPID;

    private SparkMaxConfig m_configDrive;
    private SparkMaxConfig m_configTurn;

    private RelativeEncoder m_driveEncoder;
    private RelativeEncoder m_turnEncoder;

    private CANcoder m_CANcoder;


    public SwerveModule(int driveMotorPort, int turnMotorPort, int CANCoderPort, double magnetOffset, double absoluteSensorDiscont) {
        m_driveMotor = new SparkMax(driveMotorPort, SparkLowLevel.MotorType.kBrushless);
        m_turnMotor = new SparkMax(turnMotorPort, SparkLowLevel.MotorType.kBrushless);

        m_CANcoder = new CANcoder(CANCoderPort, SwerveConstants.kCANCoderBus);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turnEncoder = m_turnMotor.getEncoder();

        m_drivePID = m_driveMotor.getClosedLoopController();
        m_turnPID = m_turnMotor.getClosedLoopController();

        m_configDrive = new SparkMaxConfig();
        m_configTurn = new SparkMaxConfig();

        m_driveMotor.configure(SwerveModuleConfigs.m_configDrive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_turnMotor.configure(SwerveModuleConfigs.m_configTurn, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        resetEncoder();

        configMagnets(magnetOffset, absoluteSensorDiscont);
        }  

    public void configure(SparkMaxConfig driveConfig, SparkMaxConfig turnConfig) {
        m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setDesiredState(SwerveModuleState targetState) {
        Rotation2d currentRotation = Rotation2d.fromDegrees(m_turnEncoder.getPosition());
        targetState.optimize(currentRotation);

        m_drivePID.setReference(targetState.speedMetersPerSecond, ControlType.kVelocity);
        m_turnPID.setReference(targetState.angle.getDegrees(), ControlType.kPosition);
    }

    public void resetEncoder() {
        m_driveEncoder.setPosition(0);
        m_turnEncoder.setPosition(m_CANcoder.getAbsolutePosition().getValueAsDouble());
    }

    public double getDistance() {
        return m_turnEncoder.getPosition();
    }

    public void configMagnets(double kCANCoderMagnetOffset, double kCANCoderAbsoluteSensorDiscontinuityPoint) {
        MagnetSensorConfigs m_magnetConfigs = new MagnetSensorConfigs();

        m_magnetConfigs
            .withMagnetOffset(kCANCoderMagnetOffset)
            .withAbsoluteSensorDiscontinuityPoint(kCANCoderAbsoluteSensorDiscontinuityPoint)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
        
        m_CANcoder.getConfigurator().apply(m_magnetConfigs);
        
    }

    public void periodic() {
        logData();
    }

    public void logData() {
        SmartDashboard.putNumber("driveEncoderPosition ", getDistance());
    }

    public SparkMaxConfigAccessor getConfigAccessor() {
        return m_driveMotor.configAccessor;
    }

}
