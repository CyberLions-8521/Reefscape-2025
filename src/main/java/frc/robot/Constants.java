package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class ControllerConstants {
        public static final double kDeadband = 0.02;
    }
    public static class SwerveConstants {
        public static final double driveP = 0;
        public static final double driveI = 0;
        public static final double driveD = 0;
        public static final double turnP = 0;
        public static final double turnI = 0;
        public static final double turnD = 0;

        public static final int turnMotorFreeLimit = 40; 
        public static final int turnMotorStallLimit = 40;
        public static final int driveMotorFreeLimit = 40;
        public static final int driveMotorStallLimit = 40;

        static final double positionWrappingUpperLimit = 360; //placeholder

    }

    public static class SwerveDrivebaseConstants {
        public static final int kBackLeftDriveID = 1;
        public static final int kBackLeftTurnID = 2;

        public static final int kBackRightDriveID = 3;
        public static final int kBackRightTurnID = 4;
    
        public static final int kFrontLeftDriveID = 5;
        public static final int kFrontLeftTurnID = 6;

        public static final int kFrontRightDriveID = 7;
        public static final int kFrontRightTurnID = 8;

        public static final double kWheelBase = Units.inchesToMeters(23.25); //x
        public static final double kTrackWidth = Units.inchesToMeters(23.25); //y

        public static final double kMaxMetersPerSecond = 5.0;
        public static final double kMaxAngularSpeed = 360;
    }
}
