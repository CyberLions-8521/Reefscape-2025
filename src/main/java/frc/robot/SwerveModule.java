// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import frc.robot.Constants.SwerveConstants;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** Add your docs here. */
public class SwerveModule {
    SparkMax m_driveMotor = new SparkMax(SwerveConstants.kDriveMotorID, SparkLowLevel.MotorType.kBrushless);
    SparkMax m_turnMotor = new SparkMax(SwerveConstants.kTurnMotorID, SparkLowLevel.MotorType.kBrushless);
    private SwerveModuleState m_swerveModuleStates = new SwerveModuleState();


}
