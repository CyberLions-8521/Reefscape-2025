package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class ControllerConstants {
        public static final double kDeadband = 0.02;
    }

    public static class SwerveConstants {
        public static double driveP = 1;
        public static double driveI = 0;
        public static double driveD = 0;
        public static double turnP = 1;
        public static double turnI = 0;
        public static double turnD = 0;

        public static double encoderPort = 0.0;

        public static final int turnMotorFreeLimit = 40; 
        public static final int turnMotorStallLimit = 40;
        public static final int driveMotorFreeLimit = 40;
        public static final int driveMotorStallLimit = 40;

        public static final double positionWrappingUpperLimit = 360; //placeholder

        public static final double kWheelDiameter = Units.inchesToMeters(4);
        public static final double kDriveGearRatio = 6.75; 
        public static final double kTurnGearRatio = (150.0 / 7.0); 
        public static final double kWheelCircumference = Math.PI * kWheelDiameter;

        public static final double kDriveConversionFactor = kWheelCircumference / kDriveGearRatio; 
        public static final double kTurnConversionFactor = 360 / kTurnGearRatio;
        
        public static final String kCANCoderBus = "ryan";

        public static final double kCANCoderMagnetOffset = 0; //config later
        public static final double kCANCoderAbsoluteSensorDiscontinuityPoint = 0; //config later

    }









    
    public static class SwerveDrivebaseConstants {
        public static final int kBackLeftDriveID = 8;
        public static final int kBackLeftTurnID = 7;

        public static final int kBackRightDriveID = 3;
        public static final int kBackRightTurnID = 14;
    
        public static final int kFrontLeftDriveID = 50;
        public static final int kFrontLeftTurnID = 2;

        public static final int kFrontRightDriveID = 4;
        public static final int kFrontRightTurnID = 6;

        public static final double kWheelBase = Units.inchesToMeters(23.25); //x
        public static final double kTrackWidth = Units.inchesToMeters(23.25); //y

        public static final double kMaxMetersPerSecond = 5.0;
        public static final double kMaxAngularSpeed = 360;

        public static final int kFrontLeftCANCoderID = 3;
        public static final int kFrontRightCANCoderID = 4;
        public static final int kBackLeftCANCoderID = 1;
        public static final int kBackRightCANCoderID = 2;

        //Angular Offsets of the modules relative to the chassis in degrees
        public static final double kFrontLeftChassisAngularOffset = -90;//NEED TO BE ASSIGNED
        public static final double kFrontRightChassisAngularOffset = 0;//NEED TO BE ASSIGNED
        public static final double kBackLeftChassisAngularOffset = 180;//NEED TO BE ASSIGNED
        public static final double kBackRightChassisAngularOffset = 90; //NEED TO BE ASSIGNED
    }
}
