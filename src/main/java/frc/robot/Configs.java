// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public final class Configs {
    public static final class SwerveModuleConfig {
        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turnConfig = new SparkMaxConfig();

        public static final MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs();

        static {
            driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
            driveConfig.encoder
                .positionConversionFactor(DriveConstants.kDriveConversionFactor)    // meters
                .velocityConversionFactor(DriveConstants.kDriveConversionFactor / 60.0);    // meters per second
            driveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD)
                .outputRange(-1.0, 1.0);

            turnConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
            turnConfig.encoder
                .positionConversionFactor(DriveConstants.kTurnConversionFactor)    // rotations of output shaft
                .velocityConversionFactor(DriveConstants.kTurnConversionFactor / 60.0);    // rotations per second
            turnConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD)
                .outputRange(-1.0, 1.0)
                .positionWrappingEnabled(true)  // allows the controller to wrap through 0 to get to setpoint
                                                        // e.g., going from 350 to 10 degrees goes through 0 instead of going all the way around
                .positionWrappingInputRange(0, 2 * Math.PI);
            
            magnetConfigs
                .withAbsoluteSensorDiscontinuityPoint(0.5)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
        }
    }
}
