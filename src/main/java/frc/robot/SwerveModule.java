// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import frc.robot.Constants.SwerveConstants;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Configs.SwerveModuleConfigs;

/** Add your docs here. */
public class SwerveModule {
    private SparkMax m_driveMotor; 
    private SparkMax m_turnMotor; 

    private SparkClosedLoopController m_drivePID;
    private SparkClosedLoopController m_turnPID;

    private SparkMaxConfig m_configDrive;
    private SparkMaxConfig m_configTurn;

    private SparkRelativeEncoder m_driveEncoder;
    private SparkRelativeEncoder m_turnEncoder;

    

    public SwerveModule(int driveMotorPort, int turnMotorPort) {
        m_driveMotor = new SparkMax(driveMotorPort, SparkLowLevel.MotorType.kBrushless);
        m_turnMotor = new SparkMax(turnMotorPort, SparkLowLevel.MotorType.kBrushless);

        m_drivePID = m_driveMotor.getClosedLoopController();
        m_turnPID = m_turnMotor.getClosedLoopController();

        m_configDrive = new SparkMaxConfig();
        m_configTurn = new SparkMaxConfig();

        m_driveMotor.configure(SwerveModuleConfigs.m_configDrive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_turnMotor.configure(SwerveModuleConfigs.m_configTurn, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_driveEncoder.setPosition(0); 
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


}
