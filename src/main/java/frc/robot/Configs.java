// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public class Configs {
    public static final class SwerveModuleConfigs {
        public static final SparkMaxConfig m_configDrive = new SparkMaxConfig();
        public static final SparkMaxConfig m_configTurn = new SparkMaxConfig();
        public static final CANcoderConfiguration m_CANcoderConfiguration = new CANcoderConfiguration();
        

        public static final CANcoderConfigurator m_frontLeftCANCoderConfigurator = new CANcoderConfigurator(new DeviceIdentifier(SwerveConstants.kFrontLeftCANCoderID, "idk", SwerveConstants.kCANCoderBus));
        public static final CANcoderConfigurator m_frontRightCANConderConfigurator = new CANcoderConfigurator(new DeviceIdentifier(SwerveConstants.kFrontRightCANCoderID, "idk", SwerveConstants.kCANCoderBus));
        public static final CANcoderConfigurator m_backLeftCANCoderConfigurator = new CANcoderConfigurator(new DeviceIdentifier(SwerveConstants.kBackLeftCANCoderID, "idk", SwerveConstants.kCANCoderBus));
        public static final CANcoderConfigurator m_backRightCANCoderConfigurator = new CANcoderConfigurator(new DeviceIdentifier(SwerveConstants.kBackRightCANCoderID, "idk", SwerveConstants.kCANCoderBus));

        static {
            m_configDrive
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .inverted(true)
                .smartCurrentLimit(SwerveConstants.driveMotorStallLimit, SwerveConstants.driveMotorFreeLimit);


            m_configTurn
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .inverted(true)
                .smartCurrentLimit(SwerveConstants.turnMotorStallLimit, SwerveConstants.turnMotorFreeLimit);

            m_configDrive.encoder
                .positionConversionFactor(SwerveConstants.kDriveConversionFactor)
                .velocityConversionFactor(SwerveConstants.kDriveConversionFactor / 60.0);

            m_configTurn.encoder
                .positionConversionFactor(SwerveConstants.kTurnConversionFactor) 
                .velocityConversionFactor(SwerveConstants.kTurnConversionFactor / 60.0);

            m_configDrive.closedLoop
                .pid(SwerveConstants.driveP, SwerveConstants.driveI, SwerveConstants.driveD)
                .outputRange(-1, 1)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, SwerveConstants.positionWrappingUpperLimit);

            m_configTurn.closedLoop
                .pid(SwerveConstants.driveP, SwerveConstants.driveI, SwerveConstants.driveD)
                .outputRange(-1, 1)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, SwerveConstants.positionWrappingUpperLimit);

        }

        public static void configureCANCoder (CANcoderConfigurator configurator, CANcoderConfiguration configurations) {
            
            configurator.apply(configurations);
        }
        
        
    }
}
