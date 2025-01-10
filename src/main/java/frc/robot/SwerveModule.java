// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import frc.robot.Constants.SwerveConstants;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public class SwerveModule {
    SparkMax m_driveMotor; 
    SparkMax m_turnMotor; 
    private SwerveModuleState m_currentState;
    private SwerveModuleState m_desiredState;
    SparkClosedLoopController m_drivePID;
    SparkClosedLoopController m_turnPID;
    SparkMaxConfig m_configDrive;
    SparkMaxConfig m_configTurn;
    

    public SwerveModule(int driveMotorPort, int turnMotorPort) {
        m_driveMotor = new SparkMax(driveMotorPort, SparkLowLevel.MotorType.kBrushless);
        m_turnMotor = new SparkMax(turnMotorPort, SparkLowLevel.MotorType.kBrushless);

        m_drivePID = m_driveMotor.getClosedLoopController();
        m_turnPID = m_turnMotor.getClosedLoopController();

        m_configDrive = new SparkMaxConfig();
        m_configTurn = new SparkMaxConfig();

        
    }

    public void configureMotors() {
        m_configDrive.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_configTurn.idleMode(SparkBaseConfig.IdleMode.kBrake);
        
        m_configDrive.inverted(true);
        m_configTurn.inverted(true);

        m_configDrive.smartCurrentLimit(SwerveConstants.driveMotorStallLimit, SwerveConstants.driveMotorFreeLimit);
        m_configTurn.smartCurrentLimit(SwerveConstants.turnMotorStallLimit, SwerveConstants.turnMotorFreeLimit);


        
        
    }

    public SwerveModuleState getSwerveModuleState() {
        return m_currentState;
    }

    public void setDesiredSwerveModuleState(SwerveModuleState desiredState) {
        m_desiredState = desiredState;
    }


}
