// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
      
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class Configs {
    public static final class MotorConfigs {
        public static final TalonFXConfiguration KRAKEN_CONFIGURATION = new TalonFXConfiguration();

        public static final SparkMaxConfig ELEV_SLAVE_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig ELEV_MASTER_CONFIG = new SparkMaxConfig();

        public static final SparkMaxConfig SHOOT_MASTER_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig SHOOT_SLAVE_CONFIG = new SparkMaxConfig();

        static {
            KRAKEN_CONFIGURATION.Slot0
                .withKP(ElevatorConstants.kP)
                .withKD(ElevatorConstants.kD)
                .withKI(ElevatorConstants.kI);

            KRAKEN_CONFIGURATION.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(80);
            
            KRAKEN_CONFIGURATION.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);

            KRAKEN_CONFIGURATION.Feedback
                .withSensorToMechanismRatio(ElevatorConstants.kGearRatio / ElevatorConstants.kGearCircumference);


            
            ELEV_MASTER_CONFIG.inverted(true)
                .idleMode(IdleMode.kBrake);
            ELEV_MASTER_CONFIG.encoder
                .positionConversionFactor(1.0 / ElevatorConstants.kGearRatio)   // rotations (output shaft)
                .velocityConversionFactor(1.0 / ElevatorConstants.kGearRatio / 60.0);   // RPS (output shaft)
            ELEV_MASTER_CONFIG.closedLoop
                .p(ElevatorConstants.kP)
                .i(ElevatorConstants.kI)
                .d(ElevatorConstants.kD);

            ELEV_SLAVE_CONFIG.follow(ElevatorConstants.kMaster, true)
                .idleMode(IdleMode.kBrake);
            ELEV_SLAVE_CONFIG.encoder.apply(ELEV_MASTER_CONFIG.encoder);
            ELEV_SLAVE_CONFIG.closedLoop.apply(ELEV_MASTER_CONFIG.closedLoop);

            SHOOT_SLAVE_CONFIG
                .follow(ShooterConstants.kMasterID, true)
                .idleMode(IdleMode.kCoast);
            
            SHOOT_MASTER_CONFIG
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
                .positionConversionFactor(SwerveConstants.kDriveConversionFactor)   // meters
                .velocityConversionFactor(SwerveConstants.kDriveConversionFactor / 60.0);   // meters per second

            m_configTurn.encoder
                .positionConversionFactor(SwerveConstants.kTurnConversionFactor)    // degrees
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
}