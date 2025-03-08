package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class ControllerConstants {
        public static final double kDeadband = 0.2;
    }

    public static class SwerveConstants {
        public static final double encoderPort = 0.0;
      
        public static final int turnMotorFreeLimit = 20; 
        public static final int turnMotorStallLimit = 20;
        public static final int driveMotorFreeLimit = 40;
        public static final int driveMotorStallLimit = 40;

        public static final double kWheelDiameter = Units.inchesToMeters(4);   //inches
        public static final double kDriveGearRatio = 6.75; 
        public static final double kTurnGearRatio = (150.0 / 7.0); 
        public static final double kWheelCircumference = Math.PI * kWheelDiameter;

        public static final double kDriveConversionFactor = kWheelCircumference / kDriveGearRatio;
        public static final double kAngleConversion = 360;
        public static final double kTurnConversionFactor = kAngleConversion / kTurnGearRatio;
        public static final double kDrivingMotorFreeSpeedRps = 5676.0 / 60.0; //neo free rpm = 5676 rpm
        public static final double kDriveWheelFreeSpeedRps = kDrivingMotorFreeSpeedRps * kDriveConversionFactor;

        public static final double kS = 0; //static constant, probably not needed
        public static final double kV = 0; //velocity constant, definitely needed
        public static final double kA = 0; //acceleration constant, probably not needed
        public static final double driveFF = 1.0 / kDriveWheelFreeSpeedRps;
        public static final double driveP = 0.025;
        public static final double driveI = 0;
        public static final double driveD = 0;
        public static final double turnP = 0.04;
        public static final double turnI = 0;
        public static final double turnD = 0.01;

        
        public static final String kCANCoderBus = "Ryan";

    }


    public static class SwerveDrivebaseConstants {

        public static final double kSlewRateLimiter = 3.0; //NOT FOR SURE YET

        public static final int kFrontLeftDriveID = 6;
        public static final int kFrontLeftTurnID = 4;

        public static final int kFrontRightDriveID = 3;
        public static final int kFrontRightTurnID = 14;

        public static final int kBackLeftDriveID = 50;
        public static final int kBackLeftTurnID = 2;

        public static final int kBackRightDriveID = 8;
        public static final int kBackRightTurnID = 7;

        //public static final int kElevatorID;

        public static final double kWheelBase = Units.inchesToMeters(23.25); //x
        public static final double kTrackWidth = Units.inchesToMeters(23.25); //y

        public static final double kMaxMetersPerSecond = 4.0;
        public static final double kMaxAngularSpeed = 2 * Math.PI; //radians

        public static final int kFrontLeftCANCoderID = 10;
        public static final int kFrontRightCANCoderID = 11;
        public static final int kBackLeftCANCoderID = 9;
        public static final int kBackRightCANCoderID = 12;

        public static final double kFrontLeftCANCoderMagnetOffset = -0.175049; //config later
        public static final double kFrontLeftCANCoderAbsoluteSensorDiscontinuityPoint = 0.5; //config later

        public static final double kFrontRightCANCoderMagnetOffset = -0.603516; //config later
        public static final double kFrontRightCANCoderAbsoluteSensorDiscontinuityPoint = 0.5; //config later

        public static final double kBackLeftCANCoderMagnetOffset = -0.645508; //config later
        public static final double kBackLeftCANCoderAbsoluteSensorDiscontinuityPoint = 0.5; //config later

        public static final double kBackRightCANCoderMagnetOffset = -0.271484; //config later
        public static final double kBackRightCANCoderAbsoluteSensorDiscontinuityPoint = 0.5; //config later
    }

    public static class ElevatorConstants {
        
        public static final int kMotor1 = 10;
        public static final int kMotor2 = 5;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kRampRate = 0.2;
        
        public static final double kGearRatio = 25;

        public static final double kResetCurrent = 0; //need to find
        public static final int kMaxCurrent = 0;//need to somehow find

        public static final double kMaxElevatorHeightMeters = 0; // Meters
        public static final double kMinElevatorHeightMeters = 0; // Meters

        public static final double kGearCircumference = Units.inchesToMeters(0);;// inches MUST BE CONFIGURED    

        public static final double kRotationToMeters = kGearCircumference / kGearRatio;
        public static final double kMaxVelocity = 5676.0;
        public static final double kMaxAcceleration = 5676.0;

        public static final double kS = 0.0;


        public static final double kG = 0;
        /*
            kG = mass * acceleration / elevator height 
         * value dependent on weight of elevator AND positon of the mechanism
         * can be estimated on the weight of elevator, mass of moving parts, and the hiehgt at which gravity is acting on the system
         */
        public static final double kV = 0;

        public static final double kA = 0;
        /*
         * determeined by performing a system test at different speeds and recording the necessary voltage to maitnain a STEADY velocity
         * higher velocity = higher value of kV
         */
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
