package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class DriveConstants {

        

        public static final double kMaxSpeedMetersPerSecond = 0; //need to be assigned
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
        public static final int kFrontLeftDriveCanID = 0; //NEED TO BE ASSIGNED
        public static final int kFrontRightDriveCanID = 0; //NEED TO BE ASSIGNED
        public static final int kBackLeftDriveCanID = 0; //NEED TO BE ASSIGNED
        public static final int kBackRightDriveCanID = 0; //NEED TO BE ASSIGNED

        public static final int kFrontLeftTurnCanID = 0; //NEED TO BE ASSIGNED
        public static final int kFrontRightTurnCanID = 0; //NEED TO BE ASSIGNED
        public static final int kBackLeftTurnCanID = 0; //NEED TO BE ASSIGNED
        public static final int kBackRightTurnCanID = 0; //NEED TO BE ASSIGNED

        public static final boolean kGyroReversed = false;
    }

    public static final class ModuleConstants { 
        

        public static final int kDrivingMotorPinionTeeth = 14;

        //Calculations required for driving motor conversion factors and feed forward
        public static final double kDriveMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRPM / 60;
        public static final double kWheelDiameterMeters = 0.1016;
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
