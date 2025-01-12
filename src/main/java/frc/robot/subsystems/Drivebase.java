package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;
import com.studica.frc.AHRS;
import java.util.function.Supplier;


import static edu.wpi.first.units.Units.Rotation;

import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivebase extends SubsystemBase {

    //Modules

    private final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveCanID, DriveConstants.kFrontLeftTurnCanID, DriveConstants.kFrontLeftChassisAngularOffset);
    private final SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveCanID, DriveConstants.kFrontRightTurnCanID, DriveConstants.kFrontRightChassisAngularOffset);
    private final SwerveModule m_backLeft = new SwerveModule(DriveConstants.kBackLeftDriveCanID, DriveConstants.kBackLeftTurnCanID, DriveConstants.kBackLeftChassisAngularOffset);
    private final SwerveModule m_backRight = new SwerveModule(DriveConstants.kBackRightDriveCanID, DriveConstants.kBackRightTurnCanID, DriveConstants.kBackRightChassisAngularOffset);

    private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    public Drivebase() {
       


    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] SwerveModuleState;
        if (fieldRelative){
            SwerveModuleState = Constants.DriveConstants.kDriveKinamatics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed, rot, Rotation2d.fromDegrees(-m_gyro.getAngle()))); 
        }
        else{
            SwerveModuleState = Constants.DriveConstants.kDriveKinamatics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        }
    
        //does more math to keep the ratios of each module keeping the speed within the maximum  (PREVENTS OVERSHOOT)
        SwerveDriveKinematics.desaturateWheelSpeeds(SwerveModuleState, DriveConstants.kMaxSpeedMetersPerSecond); 
        
        m_frontLeft.setDesiredState(SwerveModuleState[0]);
        m_frontRight.setDesiredState(SwerveModuleState[1]);
        m_backLeft.setDesiredState(SwerveModuleState[2]);
        m_backRight.setDesiredState(SwerveModuleState[3]);
    }

    public void getTwitter() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
      }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
  }


    public void resetEncoders(){
        m_frontLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_backLeft.resetEncoders();
        m_backRight.resetEncoders();
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public double getHeading() {
        return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
    }

    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

}
