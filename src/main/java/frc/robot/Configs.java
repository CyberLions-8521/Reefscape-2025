// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public final class Configs {
    public static final class SwerveModuleConfig {
        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turnConfig = new SparkMaxConfig();

        static {
            double driveConversionFactor = 1.0;
            double turnConversionFactor = 2 * Math.PI;

            driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
            driveConfig.encoder
                .positionConversionFactor(driveConversionFactor)    // meters
                .velocityConversionFactor(driveConversionFactor / 60.0);    // meters per second
            driveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(1.0, 0.0, 0.0)
                .outputRange(-1.0, 1.0);

            turnConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
            turnConfig.encoder
                .positionConversionFactor(turnConversionFactor)    // radians
                .velocityConversionFactor(turnConversionFactor / 60.0);    // radians per second
            turnConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(1.0, 0.0, 0.0)
                .outputRange(-1.0, 1.0)
                .positionWrappingEnabled(true)  // allows the controller to wrap through 0 to get to setpoint
                                                        // e.g., going from 350 to 10 degrees goes through 0 instead of going all the way around
                .positionWrappingInputRange(0, 2 * Math.PI);
        }
    }
}
