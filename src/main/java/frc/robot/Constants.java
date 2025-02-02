package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class ControllerConstants {
        public static final double kDeadband = 0.02;
    }

    public static final class DriveConstants {
        // TODO: Get CAN IDs for motors
        public static final int kFrontLeftDriveID = 1;
        public static final int kFrontLeftTurnID = 2;
        public static final int kFrontRightDriveID = 3; //placeholder
        public static final int kFrontRightTurnID = 4; //placeholder
        public static final int kBackLeftDriveID = 5;
        public static final int kBackLeftTurnID = 6;
        public static final int kBackRightDriveID = 7; //placeholder
        public static final int kBackRightTurnID = 8; //placeholder
        public static final int kFrontLeftCANcoderID = 9;
        public static final int kFrontRightCANcoderID = 10;
        public static final int kBackLeftCANcoderID = 11;
        public static final int kBackRightCANcoderID = 12;

        public static final double kFrontLeftMagnetOffset = 0.0;
        public static final double kFrontRightMagnetOffset = 0.0;
        public static final double kBackLeftMagnetOffset = 0.0;
        public static final double kBackRightMagnetOffset = 0.0;
        
        // +x direction
        public static final double kWheelBase = Units.inchesToMeters(20.0);

        // +y direction
        public static final double kTrackWidth = Units.inchesToMeters(20.0);

        public static final double kMaxSpeedMetersPerSecond = 3.0;
        public static final double kMaxAngularSpeed = 1; // rotations per second

        private static final double kDriveGearRatio = 6.75; // L2 Ratio
        private static final double kWheelDiameter = 1.0;   // meters
        public static final double kDriveConversionFactor = kWheelDiameter / kDriveGearRatio;   // meters
        private static final double kTurnGearRatio = 150.0 / 7.0;   // found in https://www.swervedrivespecialties.com/products/mk4i-swerve-module
        public static final double kTurnConversionFactor = 1.0 / kTurnGearRatio;

        public static final double kDriveP = 1.0;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kTurnP = 1.0;
        public static final double kTurnI = 0.0;
        public static final double kTurnD = 0.0;
    }
}
