// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.SwerveModuleConfigs;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveDrivebaseConstants;
import frc.robot.SwerveModule;
import frc.robot.LimelightHelpers;


public class Swerve extends SubsystemBase {
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final SwerveDriveKinematics m_kinematics;
  private final SwerveDrivePoseEstimator m_PoseEstimator;

  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private final SlewRateLimiter filter = new SlewRateLimiter(SwerveDrivebaseConstants.kSlewRateLimiter);
  
  //CONSTRUCTOR
  public Swerve() {
    m_frontLeft = new SwerveModule(
      SwerveDrivebaseConstants.kFrontLeftDriveID,
      SwerveDrivebaseConstants.kFrontLeftTurnID,
      SwerveDrivebaseConstants.kFrontLeftCANCoderID,
      SwerveDrivebaseConstants.kFrontLeftCANCoderMagnetOffset,
      SwerveDrivebaseConstants.kFrontLeftCANCoderAbsoluteSensorDiscontinuityPoint
    );

    m_frontRight = new SwerveModule(
      SwerveDrivebaseConstants.kFrontRightDriveID,
      SwerveDrivebaseConstants.kFrontRightTurnID,
      SwerveDrivebaseConstants.kFrontRightCANCoderID,
      SwerveDrivebaseConstants.kFrontRightCANCoderMagnetOffset,
      SwerveDrivebaseConstants.kFrontRightCANCoderAbsoluteSensorDiscontinuityPoint
    );

    m_backLeft = new SwerveModule(
      SwerveDrivebaseConstants.kBackLeftDriveID,
      SwerveDrivebaseConstants.kBackLeftTurnID,
      SwerveDrivebaseConstants.kBackLeftCANCoderID,
      SwerveDrivebaseConstants.kBackLeftCANCoderMagnetOffset,
      SwerveDrivebaseConstants.kBackLeftCANCoderAbsoluteSensorDiscontinuityPoint
    );

    m_backRight = new SwerveModule(
      SwerveDrivebaseConstants.kBackRightDriveID,
      SwerveDrivebaseConstants.kBackRightTurnID,
      SwerveDrivebaseConstants.kBackRightCANCoderID,
      SwerveDrivebaseConstants.kBackRightCANCoderMagnetOffset,
      SwerveDrivebaseConstants.kBackRightCANCoderAbsoluteSensorDiscontinuityPoint
    );

    m_kinematics = new SwerveDriveKinematics(
      new Translation2d(SwerveDrivebaseConstants.kWheelBase / 2, SwerveDrivebaseConstants.kTrackWidth / 2),
      new Translation2d(SwerveDrivebaseConstants.kWheelBase / 2, -SwerveDrivebaseConstants.kTrackWidth / 2),
      new Translation2d(-SwerveDrivebaseConstants.kWheelBase / 2, SwerveDrivebaseConstants.kTrackWidth / 2),
      new Translation2d(-SwerveDrivebaseConstants.kWheelBase / 2, -SwerveDrivebaseConstants.kTrackWidth / 2)
    );
    

    m_PoseEstimator = new SwerveDrivePoseEstimator(m_kinematics, Rotation2d.fromDegrees(-m_gyro.getAngle()), getModulePositions(), LimelightHelpers.getBotPose2d_wpiBlue("PUT IN REAL NAME LATER"));

    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  
    putSmartDashboard();
  }

  public void putSmartDashboard(){
    // SmartDashboard.putNumber("driveFF", SwerveConstants.driveFF);
    // SmartDashboard.putNumber("driveP", 0);
    // SmartDashboard.putNumber("driveI", 0);
    // SmartDashboard.putNumber("driveD", 0);

    //SmartDashboard.putNumber("turnP", 0);
    //SmartDashboard.putNumber("turnI", 0);
    //SmartDashboard.putNumber("turnD", 0);

  }

  
 public void drive(double vx, double vy, double omega, boolean fieldRelative) {

    SwerveModuleState[] m_swerveModuleStates;
    if(fieldRelative) {
      m_swerveModuleStates = m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Rotation2d.fromDegrees(-m_gyro.getAngle())));
    } else {
      m_swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(vx, vy, omega));
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(
        m_swerveModuleStates, SwerveDrivebaseConstants.kMaxMetersPerSecond);
    m_frontLeft.setDesiredState(m_swerveModuleStates[0]);
    m_frontRight.setDesiredState(m_swerveModuleStates[1]);
    m_backLeft.setDesiredState(m_swerveModuleStates[2]);
    m_backRight.setDesiredState(m_swerveModuleStates[3]);
  }

  public void drive(ChassisSpeeds speeds) {

    SwerveModuleState[] m_swerveModuleStates;
      m_swerveModuleStates = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        m_swerveModuleStates, SwerveDrivebaseConstants.kMaxMetersPerSecond);
    m_frontLeft.setDesiredState(m_swerveModuleStates[0]);
    m_frontRight.setDesiredState(m_swerveModuleStates[1]);
    m_backLeft.setDesiredState(m_swerveModuleStates[2]);
    m_backRight.setDesiredState(m_swerveModuleStates[3]);
  }

  //for debugging
  private void runMotors(double speed, double steer) {
    m_frontLeft.turnMotors(filter.calculate(speed), steer);
    m_frontRight.turnMotors(filter.calculate(speed), steer);
    m_backLeft.turnMotors(filter.calculate(speed), steer);
    m_backRight.turnMotors(filter.calculate(speed), steer);

  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getSwerveModulePosition(),
      m_frontRight.getSwerveModulePosition(),
      m_backLeft.getSwerveModulePosition(),
      m_backRight.getSwerveModulePosition() };
  }

  public void resetEncoders(){
    m_frontLeft.resetEncoder();
    m_frontRight.resetEncoder();
    m_backLeft.resetEncoder();
    m_backRight.resetEncoder();
  }

  public double getStraightDistance() {
    return ((m_frontLeft.getCANCoderPosition() + m_frontRight.getCANCoderPosition() + m_backLeft.getCANCoderPosition() + m_backRight.getCANCoderPosition()) / 4);
  }

  public Pose2d getPose() {
    return m_PoseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d newPose) {
    m_PoseEstimator.resetPose(newPose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(m_frontLeft.getCurrentState(), m_frontRight.getCurrentState(), m_backLeft.getCurrentState(), m_backRight.getCurrentState());
  }

  public void configureCANCoders() {
    m_frontLeft.configMagnets(SwerveDrivebaseConstants.kFrontLeftCANCoderMagnetOffset, SwerveDrivebaseConstants.kFrontLeftCANCoderAbsoluteSensorDiscontinuityPoint);
    m_frontRight.configMagnets(SwerveDrivebaseConstants.kFrontRightCANCoderMagnetOffset, SwerveDrivebaseConstants.kFrontRightCANCoderAbsoluteSensorDiscontinuityPoint);
    m_backLeft.configMagnets(SwerveDrivebaseConstants.kBackLeftCANCoderMagnetOffset, SwerveDrivebaseConstants.kBackLeftCANCoderAbsoluteSensorDiscontinuityPoint);
    m_backRight.configMagnets(SwerveDrivebaseConstants.kBackRightCANCoderMagnetOffset, SwerveDrivebaseConstants.kBackRightCANCoderAbsoluteSensorDiscontinuityPoint);
  }

  public Command resetEncodersCommand() {
    return this.runOnce(this::resetEncoders);
    // this.runOnce(lambda or function pointer)        == new InstantCommand(lambda, this)
  }

  public Command resetGyroCommand() {
    return this.runOnce(this::resetGyro);
  }

  public Command testMotorsCommand(Supplier<Double> speed, Supplier<Double> steer) {
    return this.run(() -> runMotors(speed.get(), steer.get()));
  }
 
  public void periodic() {
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if (limelightMeasurement.tagCount >= 2) {  // Only trust measurement if we see multiple tags
      m_PoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
      m_PoseEstimator.addVisionMeasurement(
        limelightMeasurement.pose,
        limelightMeasurement.timestampSeconds
      );
}
    m_PoseEstimator.update(m_gyro.getRotation2d(), getModulePositions());
  }

  public void SmartDashboardTunePID()
  {
    m_frontLeft.logData("FL");
    // m_frontRight.logData("FR");
    // m_backLeft.logData("BL");
    // m_backRight.logData("BR");
    
    // double driveFF = SmartDashboard.getNumber("driveFF", SwerveConstants.driveFF);
    // double driveP = SmartDashboard.getNumber("driveP", 0);
    // double driveI = SmartDashboard.getNumber("driveI", 0);
    // double driveD = SmartDashboard.getNumber("driveD", 0);
    double turnP = SmartDashboard.getNumber("turnP", 0);
    double turnI = SmartDashboard.getNumber("turnI", 0);
    double turnD = SmartDashboard.getNumber("turnD", 0);

    // SmartDashboard.putNumber("front left P",m_frontLeft.getConfigAccessor().closedLoop.getP());
    
    if (/*(SwerveConstants.driveFF != driveFF) ||
    (SwerveConstants.driveP != driveP) || 
    (SwerveConstants.driveI != driveI) || 
    (SwerveConstants.driveD != driveD) || */ 
    (SwerveConstants.turnP != turnP) || 
    (SwerveConstants.turnI != turnI) || 
    (SwerveConstants.turnD != turnD) ) {
      // SwerveConstants.driveFF = driveFF;
      // SwerveConstants.driveP = driveP;
      // SwerveConstants.driveI = driveI;
      // SwerveConstants.driveD = driveD;
      // SwerveConstants.turnP = turnP;
      // SwerveConstants.turnI = turnI;
      // SwerveConstants.turnD = turnD;
      
      // SwerveModuleConfigs.m_configDrive.closedLoop
      //   .pidf(driveP, driveI, driveD, driveFF);

      SwerveModuleConfigs.m_configTurn.closedLoop
        .pid(turnP, turnI, turnD);

      
      m_frontLeft.configure(SwerveModuleConfigs.m_configDrive, SwerveModuleConfigs.m_configTurn);
      m_frontRight.configure(SwerveModuleConfigs.m_configDrive, SwerveModuleConfigs.m_configTurn);
      m_backLeft.configure(SwerveModuleConfigs.m_configDrive, SwerveModuleConfigs.m_configTurn);
      m_backRight.configure(SwerveModuleConfigs.m_configDrive, SwerveModuleConfigs.m_configTurn);
        
    }
  }

}