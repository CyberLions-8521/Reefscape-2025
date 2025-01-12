package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivebase extends SubsystemBase {

    //Modules
    private final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveCanID, DriveConstants.kFrontLeftTurnCanID, DriveConstants.kFrontLeftChassisAngularOffset);
    private final SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveCanID, DriveConstants.kFrontRightTurnCanID, DriveConstants.kFrontRightChassisAngularOffset);
    private final SwerveModule m_backLeft = new SwerveModule(DriveConstants.kBackLeftDriveCanID, DriveConstants.kBackLeftTurnCanID, DriveConstants.kBackLeftChassisAngularOffset);
    private final SwerveModule m_backRight = new SwerveModule(DriveConstants.kBackRightDriveCanID, DriveConstants.kBackRightTurnCanID, DriveConstants.kBackRightChassisAngularOffset);

    public Drivebase() {
       


    }


    public void resetEncoders(){
        m_frontLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_backLeft.resetEncoders();
        m_backRight.resetEncoders();
    }
    
}
