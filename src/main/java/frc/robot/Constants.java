package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class ControllerConstants {
        public static final double kDeadband = 0.2;
    }

    public static final class DriveConstants {
        public static final int kFrontLeftDriveID  = 50;
        public static final int kFrontLeftTurnID   = 2;
        public static final int kFrontRightDriveID = 4;
        public static final int kFrontRightTurnID  = 6;
        public static final int kBackLeftDriveID   = 8;
        public static final int kBackLeftTurnID    = 7;
        public static final int kBackRightDriveID  = 3;
        public static final int kBackRightTurnID   = 14;

        public static final int kFrontLeftCANcoderID  = 10;
        public static final int kFrontRightCANcoderID = 11;
        public static final int kBackLeftCANcoderID   = 9;
        public static final int kBackRightCANcoderID  = 12;

        public static final int kDriveSmartCurrentLimit = 50;    // amps
        public static final int kTurnSmartCurrentLimit  = 50;    // amps

        public static final double kFrontLeftMagnetOffset  = -0.179931640625;
        public static final double kFrontRightMagnetOffset = 0.396728515625;
        public static final double kBackLeftMagnetOffset   = 0.35595703125;
        public static final double kBackRightMagnetOffset  = -0.286376953125;

        // +x direction
        public static final double kWheelBase = Units.inchesToMeters(23.25);

        // +y direction
        public static final double kTrackWidth = Units.inchesToMeters(23.25);

        public static final double kMaxSpeedMetersPerSecond = 3.0;
        public static final double kMaxAngularSpeed = 1; // rotations per second

        private static final double kDriveGearRatio = 6.75; // L2 Ratio
        private static final double kWheelDiameter  = Units.inchesToMeters(4);
        public  static final double kDriveConversionFactor = kWheelDiameter / kDriveGearRatio;   // meters

        private static final double kTurnGearRatio = 150.0 / 7.0;   // found in https://www.swervedrivespecialties.com/products/mk4i-swerve-module
        public  static final double kTurnConversionFactor = 1.0 / kTurnGearRatio;

        public static final double kDriveP = 1.0;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;

        public static final double kTurnP = 1.0;
        public static final double kTurnI = 0.0;
        public static final double kTurnD = 0.0;
    }
}
