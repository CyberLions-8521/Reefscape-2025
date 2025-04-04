// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class Configs {

    public static final class AlgaeConfig {
        public static final SparkMaxConfig kAlgaeConfig = new SparkMaxConfig();

        static {
            kAlgaeConfig
                .idleMode(IdleMode.kBrake).smartCurrentLimit(40, 40);
        }
    }
    public static final class ElevatorConfigs {
        public static final TalonFXConfiguration kKrakenConfig = new TalonFXConfiguration();

        public static final SparkMaxConfig kMasterConfig = new SparkMaxConfig();
        public static final SparkMaxConfig kSlaveConfig = new SparkMaxConfig();

        static {
            kKrakenConfig.Slot0
                .withKP(ElevatorConstants.kP)
                .withKD(ElevatorConstants.kD)
                .withKI(ElevatorConstants.kI);
            kKrakenConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(80);
            kKrakenConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);
            kKrakenConfig.Feedback
                .withSensorToMechanismRatio(ElevatorConstants.kGearRatio / ElevatorConstants.kGearCircumference);
            
            kMasterConfig.inverted(true)
                .idleMode(IdleMode.kBrake);
            kMasterConfig.encoder
                .positionConversionFactor(1.0 / ElevatorConstants.kGearRatio)   // rotations (output shaft)
                .velocityConversionFactor(1.0 / ElevatorConstants.kGearRatio / 60.0);   // RPS (output shaft)
            kMasterConfig.closedLoop
                .p(ElevatorConstants.kP)
                .i(ElevatorConstants.kI)
                .d(ElevatorConstants.kD);

            kSlaveConfig.follow(ElevatorConstants.kMaster, true)
                .idleMode(IdleMode.kBrake);
            kSlaveConfig.encoder.apply(kMasterConfig.encoder);
            kSlaveConfig.closedLoop.apply(kMasterConfig.closedLoop);
        }
    }

    public static final class ShooterConfigs {
        public static final SparkMaxConfig kMasterConfig = new SparkMaxConfig();
        public static final SparkMaxConfig kSlaveConfig  = new SparkMaxConfig();

        static {
            kMasterConfig
                .idleMode(IdleMode.kCoast);
            kSlaveConfig
                .follow(ShooterConstants.kMasterID, true)
                .idleMode(IdleMode.kCoast);
        }
    }
  
    public static final class SwerveModuleConfigs {
        public static final SparkMaxConfig m_configDrive = new SparkMaxConfig();
        public static final SparkMaxConfig m_configTurn = new SparkMaxConfig();

        static {
            m_configDrive
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(SwerveConstants.driveMotorStallLimit, SwerveConstants.driveMotorFreeLimit);

            m_configTurn
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(SwerveConstants.turnMotorStallLimit, SwerveConstants.turnMotorFreeLimit);

            m_configDrive.encoder
                .positionConversionFactor(SwerveConstants.kDriveConversionFactor)           // meters
                .velocityConversionFactor(SwerveConstants.kDriveConversionFactor / 60.0);   // meters per second

            m_configTurn.encoder
                .positionConversionFactor(SwerveConstants.kTurnConversionFactor)            // degrees
                .velocityConversionFactor(SwerveConstants.kTurnConversionFactor / 60.0);    // degrees per second

            m_configDrive.closedLoop
                .pidf(SwerveConstants.driveP, SwerveConstants.driveI, SwerveConstants.driveD, SwerveConstants.driveFF)
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

    public static final class ClimberConfigs {
        public static final TalonFXConfiguration kKrakenConfig = new TalonFXConfiguration();

        static {
            kKrakenConfig.Slot0
                .withKP(ElevatorConstants.kP)
                .withKD(ElevatorConstants.kD)
                .withKI(ElevatorConstants.kI);
            kKrakenConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(80);
            kKrakenConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);
            kKrakenConfig.Feedback
                .withSensorToMechanismRatio(ElevatorConstants.kGearRatio / ElevatorConstants.kGearCircumference);
        }
    }
}