package frc.robot;

import frc.robot.Configs.SwerveModuleConfig;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private final SparkMax m_driveMotor;
    private final SparkMax m_turnMotor;
    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_turnEncoder;
    private final SparkClosedLoopController m_drivePID;
    private final SparkClosedLoopController m_turnPID;
    private final CANcoder m_canCoder;
    private SwerveModuleState m_desiredState = new SwerveModuleState();    // for debugging purposes

    public SwerveModule(int driveID, int turnID, int canCoderID, double magnetOffset) {
        m_driveMotor   = new SparkMax(driveID, MotorType.kBrushless);
        m_driveEncoder = m_driveMotor.getEncoder();
        m_drivePID     = m_driveMotor.getClosedLoopController();
        m_driveEncoder.setPosition(0);
        m_driveMotor.configure(SwerveModuleConfig.driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_turnMotor   = new SparkMax(turnID, MotorType.kBrushless);
        m_turnEncoder = m_turnMotor.getEncoder();
        m_turnPID     = m_turnMotor.getClosedLoopController();
        m_turnMotor.configure(SwerveModuleConfig.turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_canCoder = new CANcoder(canCoderID, "rio");
        m_canCoder.getConfigurator().apply(SwerveModuleConfig.magnetConfigs.withMagnetOffset(magnetOffset));
        
        reCalibrateTurnEncoder();
    }

    public REVLibError resetDriveEncoder() {
        return m_driveEncoder.setPosition(0.0);
    }

    public REVLibError reCalibrateTurnEncoder() {
        return m_turnEncoder.setPosition(m_canCoder.getAbsolutePosition().getValueAsDouble());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentRotation = Rotation2d.fromRotations(m_turnEncoder.getPosition());
        // Optimize to allow module to make the smallest turn to arrive at the rotational setpoint
        // e.g., it will turn through 0 degrees when going from 350 degrees to 10 degrees
        desiredState.optimize(currentRotation);

        // Taken from WPILib example; comment out if it doesn't work
        desiredState.cosineScale(currentRotation);

        m_drivePID.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
        m_turnPID.setReference(desiredState.angle.getRotations() , ControlType.kPosition);

        m_desiredState = desiredState;  // for debugging purposes
    }

    public void testMotors(double drive, double turn) {
        m_driveMotor.set(drive);
        m_turnMotor.set(turn);
    }

    /*
     * All methods below are for dynamic PID tuning in SmartDashboard
     */
    public void logData(String description) {
        SmartDashboard.putNumber(String.format("%5s Drive Encoder", description), m_driveEncoder.getPosition());
        SmartDashboard.putNumber(String.format("%6s Turn Encoder" , description), m_turnEncoder.getPosition());
        SmartDashboard.putNumber(String.format("%10s CAN-coder"    , description), m_canCoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber(String.format("%5s Desired Angle", description), m_desiredState.angle.getDegrees());
        SmartDashboard.putNumber(String.format("%5s Desired Speed", description), m_desiredState.speedMetersPerSecond);
    }

    public void logPID() {
        logPID("");
    }

    public void logPID(String description) {
        SmartDashboard.putNumber(description + "driveP", m_driveMotor.configAccessor.closedLoop.getP());
        SmartDashboard.putNumber(description + "driveI", m_driveMotor.configAccessor.closedLoop.getI());
        SmartDashboard.putNumber(description + "driveD", m_driveMotor.configAccessor.closedLoop.getD());
        SmartDashboard.putNumber(description + "turnP", m_turnMotor.configAccessor.closedLoop.getP());
        SmartDashboard.putNumber(description + "turnI", m_turnMotor.configAccessor.closedLoop.getI());
        SmartDashboard.putNumber(description + "turnD", m_turnMotor.configAccessor.closedLoop.getD());
    }

    public void configureDriveMotor() {
        m_driveMotor.configure(SwerveModuleConfig.driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void configureTurnMotor() {
        m_turnMotor.configure(SwerveModuleConfig.turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}