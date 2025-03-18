package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class ControllerConstants {
        public static final double kDeadband = 0.2;
    }

    public static class SwerveConstants {
        public static final int turnMotorFreeLimit = 20;    // current limits in amps
        public static final int turnMotorStallLimit = 20;   // current limits in amps
        public static final int driveMotorFreeLimit = 40;   // current limits in amps
        public static final int driveMotorStallLimit = 40;  // current limits in amps

        private static final double kWheelDiameter = Units.inchesToMeters(4);
        private static final double kWheelCircumference = Math.PI * kWheelDiameter;
        private static final double kDriveGearRatio = 6.75;     // found on SDS page for MK4i
        public static final double kDriveConversionFactor = kWheelCircumference / kDriveGearRatio;  // meters (of robot travel)

        private static final double kTurnGearRatio = (150.0 / 7.0);     // found on SDS page for MK4i
        public static final double kAngleConversion = 360;              // degrees
        public static final double kTurnConversionFactor = kAngleConversion / kTurnGearRatio;       // degrees (of output shaft)

        private static final double kDrivingMotorFreeSpeedRps = 5676.0 / 60.0;      // neo free rpm = 5676 rpm
        private static final double kDriveWheelFreeSpeedRps = kDrivingMotorFreeSpeedRps * kDriveConversionFactor;
        public static final double driveFF = 1.0 / kDriveWheelFreeSpeedRps;
        public static final double driveP = 0.025;
        public static final double driveI = 0;
        public static final double driveD = 0;
        public static final double turnP = 0.04;
        public static final double turnI = 0;
        public static final double turnD = 0.01;        
        public static final String kCANCoderBus = "Ryan";   // name assigned in Phoenix Tuner X
    }


    public static class SwerveDrivebaseConstants {
        public static final double kSlewRateLimiter = 3.0;
        public static final int kFrontLeftDriveID  = 6;
        public static final int kFrontLeftTurnID   = 4;
        public static final int kFrontRightDriveID = 3;
        public static final int kFrontRightTurnID  = 14;
        public static final int kBackLeftDriveID   = 50;
        public static final int kBackLeftTurnID    = 2;
        public static final int kBackRightDriveID  = 8;
        public static final int kBackRightTurnID   = 7;

        // Note: CANcoder CAN IDs are on a separate CAN bus than SparkMAXs
        // allowing for duplicates between CANcoders and SparkMAXs
        public static final int kFrontLeftCANCoderID  = 10;
        public static final int kFrontRightCANCoderID = 11;
        public static final int kBackLeftCANCoderID   = 9;
        public static final int kBackRightCANCoderID  = 12;

        public static final double kWheelBase = Units.inchesToMeters(23.25);    // x-direction of robot
        public static final double kTrackWidth = Units.inchesToMeters(23.25);   // y-direction of robot

        public static final double kMaxMetersPerSecond = 4.0;
        public static final double kMaxAngularSpeed = 2 * Math.PI;  // radians

        public static final double kCANcoderAbsDiscontPoint = 0.5;
        public static final double kFrontLeftCANCoderMagnetOffset  = -0.175049;     // measured in Phoenix Tuner X
        public static final double kFrontRightCANCoderMagnetOffset = -0.603516;     // measured in Phoenix Tuner X
        public static final double kBackLeftCANCoderMagnetOffset   = -0.645508;     // measured in Phoenix Tuner X
        public static final double kBackRightCANCoderMagnetOffset  = -0.271484;     // measured in Phoenix Tuner X

        public static final double kStrafeP = 0.0;
        public static final double kStrafeI = 0.0;
        public static final double kStrafeD = 0.0;
    }

    public static class ElevatorConstants {
        public static final int kMaster = 10;
        public static final int kSlave = 5;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kRampRate = 0.2;
        public static final double kGearRatio = 25;
        public static final double kResetCurrent = 0;   // TODO: need to find
        public static final int kMaxCurrent = 0;        // TODO: need to somehow find
        public static final double kMaxHeight = 6.00;   // Output revolutions
        public static final double kMinHeight = 0;      // Output revolutions
        public static final double kGearCircumference = Units.inchesToMeters(0);    // meters TODO: MUST BE CONFIGURED    
        public static final double kRotationToMeters = kGearCircumference / kGearRatio;
        public static final double kMaxVelocity = 0.1;
        public static final double kMaxAcceleration = 0.1;
        public static final double kS = 0.0;
        public static final double kG = 0.28;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kL1Setpoint = 0; //NEEDS TO BE SET!!!!!!!!!!!!!!!!!!!
        public static final double kL2Setpoint = 0; //NEEDS TO BE SET!!!!!!!!!!!!!!!!!!!
        public static final double kL3Setpoint = 0; //NEEDS TO BE SET!!!!!!!!!!!!!!!!!!!
        public static final double kL4Setpoint = 0; //NEEDS TO BE SET!!!!!!!!!!!!!!!!!!!
    }

    public static class OperaterConstants {
        public static final int kDriveControllerPort = 0;
        public static final int kCommandControllerPort = 1;
    }

    public static class ShooterConstants {
        public static final int kMasterID = 25;
        public static final int kSlaveID = 17;
    }
}