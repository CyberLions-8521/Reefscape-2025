// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
      
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class Configs {
    public static final class ShooterConfigs {

        public static final SparkMaxConfig SHOOT_MASTER_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig SHOOT_SLAVE_CONFIG = new SparkMaxConfig();

        static {
            SHOOT_SLAVE_CONFIG
                .follow(ShooterConstants.kMasterID, true)
                .idleMode(IdleMode.kCoast);
            
            SHOOT_MASTER_CONFIG
                .idleMode(IdleMode.kCoast);
        }
    }

    public static final class ElevatorConfigs {
        public static final SparkMaxConfig ELEV_SLAVE_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig ELEV_MASTER_CONFIG = new SparkMaxConfig();

        static {
            m_masterConfig.encoder
                    .positionConversionFactor(Constants.ElevatorConstants.kRotationToMeters) // Converts Rotations to Meters
                    .velocityConversionFactor(Constants.ElevatorConstants..kRotationToMeters / 60); // Converts RPM to MPS
            m_masterConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD, ClosedLoopSlot.kSlot0)//Change PID with these constants.                    .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
            
            
            m_masterConfig.idleMode(IdleMode.kBrake);
            m_slaveConfig.idleMode(IdleMode.kBrake);

            m_masterConfig.smartCurrentLimit(Constants.ElevatorConstants.kMaxCurrent);
            m_slaveConfig.smartCurrentLimit(Constants.ElevatorConstants.kMaxCurrent);

            m_masterConfig.closedLoopRampRate(Constants.ElevatorConstants.kRampRate);
            m_slaveConfig.closedLoopRampRate(Constants.ElevatorConstants.kRampRate);
        }
    }
  
    public static final class SwerveModuleConfigs {
        public static final SparkMaxConfig m_masterConfig = new SparkMaxConfig();
        public static final SparkMaxConfig m_slaveConfig = new SparkMaxConfig();



        static {

            // double drivingVelocityFeedForward = 1 / SwerveConstants.kDrivingMotorFreeSpeedRps;
            m_configDrive
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(SwerveConstants.driveMotorStallLimit, SwerveConstants.driveMotorFreeLimit);

            m_configTurn
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(SwerveConstants.turnMotorStallLimit, SwerveConstants.turnMotorFreeLimit);

            m_configDrive.encoder
                .positionConversionFactor(SwerveConstants.kDriveConversionFactor)
                .velocityConversionFactor(SwerveConstants.kDriveConversionFactor / 60.0);

            m_configTurn.encoder
                .positionConversionFactor(SwerveConstants.kTurnConversionFactor) 
                .velocityConversionFactor(SwerveConstants.kTurnConversionFactor / 60.0);

            m_configDrive.closedLoop
                .pidf(SwerveConstants.driveP, SwerveConstants.driveI, SwerveConstants.driveD, SwerveConstants.driveFF)
                //drivingVelocityFeedForward1
                .outputRange(-1, 1)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(false);

            m_configTurn.closedLoop
                .pid(SwerveConstants.driveP, SwerveConstants.driveI, SwerveConstants.driveD)
                .outputRange(-1, 1)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-SwerveConstants.kAngleConversion / 2.0, SwerveConstants.kAngleConversion / 2.0); 


        }
    }
}