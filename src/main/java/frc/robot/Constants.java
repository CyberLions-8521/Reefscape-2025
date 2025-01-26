package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class ControllerConstants {
        public static final double kDeadBand = 0.02;    
        public static final int kDriverControllerPort = 0; //NEED TO BE ASSIGNED
    }

    public static final class DriveConstants {

        public static double driveP = 1;
        public static double driveI = 0;
        public static double driveD = 0;
        public static double turnP = 1;
        public static double turnI = 0;
        public static double turnD = 0;


        public static final int turnMotorFreeLimit = 40;
        public static final int turnMotorStallLimit = 40;
        public static final int driveMotorFreeLimit = 40;
        public static final int driveMotorStallLimit = 40;
        
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAngularSpeed = 2 * Math.PI;

        //Chassis Configuration

        public static final double kTrackWidth = Units.inchesToMeters(23.25);
        //Distance between centers of right and left wheels on robot

        public static final double kWheelBase = Units.inchesToMeters(23.25);
        //Distance between front and back wheels on robot

        public static final SwerveDriveKinematics kDriveKinamatics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase/2, kTrackWidth/2),
            new Translation2d(kWheelBase/2, -kTrackWidth/2),
            new Translation2d(-kWheelBase/2, kTrackWidth/2),
            new Translation2d(-kWheelBase/2, -kTrackWidth/2));

        //Angular Offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;//NEED TO BE ASSIGNED
        public static final double kFrontRightChassisAngularOffset = 0;//NEED TO BE ASSIGNED
        public static final double kBackLeftChassisAngularOffset = Math.PI;//NEED TO BE ASSIGNED
        public static final double kBackRightChassisAngularOffset = Math.PI / 2; //NEED TO BE ASSIGNED

        //SparkMax CAN IDs
        public static final int kBackLeftDriveID = 8;
        public static final int kBackLeftTurnID = 7;

        public static final int kBackRightDriveID = 3;
        public static final int kBackRightTurnID = 14;

        public static final int kFrontLeftDriveID = 50;
        public static final int kFrontLeftTurnID = 2;

        public static final int kFrontRightDriveID = 4;
        public static final int kFrontRightTurnID = 6;

        public static final boolean kGyroReversed = false;
        
        public static final double kDriveGearRatio = 6.75;
        public static final double kTurnGearRatio = (150.0 / 7.0);


        public static final int kDrivingMotorPinionTeeth = 14;

        //Calculations required for driving motor conversion factors and feed forward
        public static final double kDriveMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRPM / 60;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
        public static final double kWheelCircumferenceMeters = Math.PI * kWheelDiameterMeters;

        // __ teeth on the wheel's bevel gear, __ teeth on the first-stage spur gear, __ teeth on the bevel pinion
        public static final double kDriveMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15); //NEED TO BE ASSIGNED
        public static final double kDriveWheelFreeSpeedRps = (kDriveMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDriveMotorReduction;
        
    }

    public static final class QIConstants {

    }

    public static final class AutoConstants {

    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRPM = 5676;
    }
}
