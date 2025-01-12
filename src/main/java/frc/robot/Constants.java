package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class ControllerConstants {
        public static final double kDeadband = 0.02;
    }

    public static final class DriveConstants {
        // TODO: Get CAN IDs for motors
        public static final int kFrontLeftDriveID = 0;
        public static final int kFrontLeftTurnID = 0;
        public static final int kFrontRightDriveID = 0; //placeholder
        public static final int kFrontRightTurnID = 1; //placeholder
        public static final int kBackLeftDriveID = 0;
        public static final int kBackLeftTurnID = 0;
        public static final int kBackRightDriveID = 0; //placeholder
        public static final int kBackRightTurnID = 1; //placeholder
        
        // +x direction
        public static final double kWheelBase = Units.inchesToMeters(20.0);

        // +y direction
        public static final double kTrackWidth = Units.inchesToMeters(20.0);

        public static final double kMaxSpeedMetersPerSecond = 3.0;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    }
}
