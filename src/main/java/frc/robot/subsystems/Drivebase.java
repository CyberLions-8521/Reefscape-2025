package frc.robot.subsystems;

import java.util.function.Supplier;

import com.studica.frc.AHRS;

import frc.robot.Constants.DriveConstants;
import frc.robot.Configs.SwerveModuleConfig;    // for PID tuning purposes
import frc.robot.SwerveModule;
// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivebase extends SubsystemBase {
    // Gyro is a NavX plugged into RoboRIO MXP SPI bus port
    private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    private final SwerveModule m_frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveID,
        DriveConstants.kFrontLeftTurnID,
        DriveConstants.kFrontLeftCANcoderID,
        DriveConstants.kFrontLeftMagnetOffset);
    private final SwerveModule m_frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveID,
        DriveConstants.kFrontRightTurnID,
        DriveConstants.kFrontRightCANcoderID,
        DriveConstants.kFrontRightMagnetOffset);
    private final SwerveModule m_backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveID,
        DriveConstants.kBackLeftTurnID,
        DriveConstants.kBackLeftCANcoderID,
        DriveConstants.kBackLeftMagnetOffset);
    private final SwerveModule m_backRight = new SwerveModule(
        DriveConstants.kBackRightDriveID,
        DriveConstants.kBackRightTurnID,
        DriveConstants.kBackRightCANcoderID,
        DriveConstants.kBackRightMagnetOffset);

    // Given as frontLeft, frontRight, backLeft, backRight
    // +x is forwards relative to the robot
    // +y is left relative to the robot when the front of the robot is facing away from you
    private final SwerveDriveKinematics m_driveKinematics = new SwerveDriveKinematics(
        new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2),
        new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2),
        new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2),
        new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2));

    public Drivebase() {}

    /**
   * Method to drive the robot using joystick info. Taken and modified from REVrobotics
   * MAXSwerve-Java-Template https://github.com/REVrobotics/MAXSwerve-Java-Template
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates;

        // Using the SwerveDriveKinematics object, convert a ChassisSpeeds into SwerveModuleStates
        if (fieldRelative) {
            swerveModuleStates = m_driveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-m_gyro.getAngle())));
        } else {
            swerveModuleStates = m_driveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        }
        
        // Desaturate wheel speeds to keep speed ratios the same while ensuring that none of the speeds
        // go above the maximum physical speed given by the hardware
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public Command resetDriveEncodersCommand() {
        return this.runOnce(this::resetDriveEncoders);
    }

    public void resetDriveEncoders() {
        m_frontLeft.resetDriveEncoder();
        m_frontRight.resetDriveEncoder();
        m_backLeft.resetDriveEncoder();
        m_backRight.resetDriveEncoder();
    }

    public Command reCalibrateTurnEncodersCommand() {
        return this.runOnce(this::reCalibrateTurnEncoders);
    }

    public void reCalibrateTurnEncoders() {
        m_frontLeft.reCalibrateTurnEncoder();
        m_frontRight.reCalibrateTurnEncoder();
        m_backLeft.reCalibrateTurnEncoder();
        m_backRight.reCalibrateTurnEncoder();
    }

    public Command testMotorsCommand(Supplier<Double> speed, Supplier<Double> steer) {
        return this.run(() -> testMotors(speed.get(), steer.get()));
    }

    private void testMotors(double speed, double steer) {
        m_frontLeft.testMotors(speed, steer);
        m_frontRight.testMotors(speed, steer);
        m_backLeft.testMotors(speed, steer);
        m_backRight.testMotors(speed, steer);
    }

    @Override
    public void periodic() {
        tunePIDSmartDashboard();
    }

    // As of Feb 9, 2025: Until examples get udated, see the Position Closed Loop Control example project in
    // https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master/Java
    private void tunePIDSmartDashboard() {
        m_frontLeft.logData("FL");  // logs encoder, cancoder, and desired state values
        m_frontRight.logData("FR");
        m_backLeft.logData("BL");
        m_backRight.logData("BR");
        m_frontLeft.logPID();   // logs pid values from the module
        double driveP = SmartDashboard.getNumber("driveP", 0);
        double driveI = SmartDashboard.getNumber("driveI", 0);
        double driveD = SmartDashboard.getNumber("driveD", 0);
        double turnP = SmartDashboard.getNumber("turnP", 0);
        double turnI = SmartDashboard.getNumber("turnI", 0);
        double turnD = SmartDashboard.getNumber("turnD", 0);
        if (driveP != DriveConstants.kDriveP || driveI != DriveConstants.kDriveI || driveD != DriveConstants.kDriveD) {
            DriveConstants.kDriveP = driveP;
            DriveConstants.kDriveI = driveI;
            DriveConstants.kDriveD = driveD;
            SwerveModuleConfig.driveConfig.closedLoop.pid(driveP, driveI, driveD);
            m_frontLeft.configureDriveMotor();
            m_frontRight.configureDriveMotor();
            m_backLeft.configureDriveMotor();
            m_backRight.configureDriveMotor();
        }

        if (turnP != DriveConstants.kTurnP || turnI != DriveConstants.kTurnI || turnD != DriveConstants.kTurnD) {
            DriveConstants.kTurnP = turnP;
            DriveConstants.kTurnI = turnI;
            DriveConstants.kTurnD = turnD;            
            SwerveModuleConfig.turnConfig.closedLoop.pid(turnP, turnI, turnD);
            m_frontLeft.configureTurnMotor();
            m_frontRight.configureTurnMotor();
            m_backLeft.configureTurnMotor();
            m_backRight.configureTurnMotor();
        }
    }
}
