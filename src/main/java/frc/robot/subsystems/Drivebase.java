package frc.robot.subsystems;

// import java.util.function.Supplier;

import com.studica.frc.AHRS;

import frc.robot.Constants.DriveConstants;
import frc.robot.SwerveModule;
// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivebase extends SubsystemBase {
    // Gyro is a NavX plugged into RoboRIO MXP SPI bus port
    private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    private final SwerveModule m_frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveID,
        DriveConstants.kFrontLeftTurnID);
    private final SwerveModule m_frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveID,
        DriveConstants.kFrontRightTurnID);
    private final SwerveModule m_backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveID,
        DriveConstants.kBackLeftTurnID);
    private final SwerveModule m_backRight = new SwerveModule(
        DriveConstants.kBackRightDriveID,
        DriveConstants.kBackRightTurnID);

    // Given as frontLeft, frontRight, backLeft, backRight
    private final SwerveDriveKinematics m_driveKinematics = new SwerveDriveKinematics(
        new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2),
        new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2),
        new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2),
        new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2));

    public Drivebase() {}

    /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates;
        if (fieldRelative) {
            swerveModuleStates = m_driveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-m_gyro.getAngle())));
        } else {
            swerveModuleStates = m_driveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        }
        
        // var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        //     fieldRelative
        //         ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
        //             Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
        //         : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }
}
