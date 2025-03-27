
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs.SwerveModuleConfigs;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveModule {
    private SparkMax m_driveMotor;
    private SparkMax m_turnMotor; 
    private RelativeEncoder m_driveEncoder;
    private RelativeEncoder m_turnEncoder;
    private SparkClosedLoopController m_drivePID;
    private SparkClosedLoopController m_turnPID;

    private CANcoder m_CANcoder;

    private SwerveModuleState m_desiredState = new SwerveModuleState();


    public SwerveModule(int driveMotorPort, int turnMotorPort, int CANCoderPort, double magnetOffset, double absoluteSensorDiscont) {
        m_driveMotor = new SparkMax(driveMotorPort, SparkLowLevel.MotorType.kBrushless);
        m_turnMotor  = new SparkMax(turnMotorPort, SparkLowLevel.MotorType.kBrushless);
        m_driveEncoder = m_driveMotor.getEncoder();
        m_turnEncoder  = m_turnMotor.getEncoder();
        m_drivePID = m_driveMotor.getClosedLoopController();
        m_turnPID  = m_turnMotor.getClosedLoopController();
        m_CANcoder = new CANcoder(CANCoderPort, SwerveConstants.kCANCoderBus);

        configure(SwerveModuleConfigs.m_configDrive, SwerveModuleConfigs.m_configTurn);
        resetEncoder();
        configMagnets(magnetOffset, absoluteSensorDiscont);
    }  

    //DATA LOGGING
    //for literally everything 
    public void logData(String motor){
        SmartDashboard.putNumber(motor + " turn position", m_turnEncoder.getPosition() % 360 - 180);
        SmartDashboard.putNumber(motor + " CANcoder", m_CANcoder.getAbsolutePosition().getValueAsDouble()*SwerveConstants.kAngleConversion);
        SmartDashboard.putNumber(motor + " desired position", m_desiredState.angle.getDegrees());

        SmartDashboard.putNumber(motor + " drive position", m_driveEncoder.getPosition());
        SmartDashboard.putNumber(motor + " turn position", m_turnEncoder.getPosition());

        SmartDashboard.putNumber(motor + " drive velocity", m_driveEncoder.getVelocity());
        SmartDashboard.putNumber(motor + " turn velocity", m_turnEncoder.getVelocity());

        SmartDashboard.putNumber(motor + " drive current", m_driveMotor.getOutputCurrent());
        SmartDashboard.putNumber(motor + " turn current", m_turnMotor.getOutputCurrent());

        SmartDashboard.putNumber(motor + " drive voltage", m_driveMotor.getAppliedOutput());
        SmartDashboard.putNumber(motor + " turn voltage", m_turnMotor.getAppliedOutput());

        SmartDashboard.putNumber(motor + " drive temp", m_driveMotor.getMotorTemperature());
        SmartDashboard.putNumber(motor + " turn temp", m_turnMotor.getMotorTemperature());

        SmartDashboard.putNumber(motor + " drive voltage", m_driveMotor.getBusVoltage());
        SmartDashboard.putNumber(motor + " turn voltage", m_turnMotor.getBusVoltage());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(), Rotation2d.fromDegrees(m_turnEncoder.getPosition()));
    }
    
    //extra tuning if needed
    public void updateFromDashboard(String motor) {
        double driveSpeed = SmartDashboard.getNumber(motor + " drive speed", 0.0);
        double turnAngle = SmartDashboard.getNumber(motor + " turn angle", 0.0);

        SwerveModuleState targetState = new SwerveModuleState(driveSpeed, Rotation2d.fromDegrees(turnAngle));
        setDesiredState(targetState);
    }

    public void zeroTurnEncoder() {
        m_turnEncoder.setPosition(0);
    }

    public void zeroDriveEncoder() {
        m_driveEncoder.setPosition(0);
    }


    public void configure(SparkMaxConfig driveConfig, SparkMaxConfig turnConfig) {
        m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setDesiredState(SwerveModuleState targetState) {
        Rotation2d currentRotation = Rotation2d.fromDegrees(m_turnEncoder.getPosition());
        targetState.optimize(currentRotation);

        m_drivePID.setReference(targetState.speedMetersPerSecond, ControlType.kVelocity);
        m_turnPID.setReference(targetState.angle.getDegrees(), ControlType.kPosition);

        m_desiredState = targetState; 
    }

    public void resetEncoder() {
        m_driveEncoder.setPosition(0);
        m_turnEncoder.setPosition(m_CANcoder.getAbsolutePosition().getValueAsDouble() * (SwerveConstants.kAngleConversion));  //degrees
    }

    public void stop() {
        m_driveMotor.set(0);
        m_turnMotor.set(0);
    }

    public void turnMotors(double speed, double steer) {
        m_driveMotor.set(speed);
        m_turnMotor.set(steer);
    }

    public double getDriveDistance() {
        return m_driveEncoder.getPosition(); // meters
    }

    //for smartdashboard logging purposes
    public double getCANCoderPosition() {
        return m_CANcoder.getAbsolutePosition().getValueAsDouble(); // rotations 
    }

    public void configMagnets(double kCANCoderMagnetOffset, double kCANCoderAbsoluteSensorDiscontinuityPoint) {
        MagnetSensorConfigs m_magnetConfigs = new MagnetSensorConfigs();

        m_magnetConfigs
            .withMagnetOffset(kCANCoderMagnetOffset)
            .withAbsoluteSensorDiscontinuityPoint(kCANCoderAbsoluteSensorDiscontinuityPoint)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
         
        m_CANcoder.getConfigurator().apply(m_magnetConfigs);
        
    }

    public SparkMaxConfigAccessor getConfigAccessor() {
        return m_driveMotor.configAccessor;
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(), Rotation2d.fromDegrees(m_turnEncoder.getPosition()));
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(m_driveEncoder.getPosition(), Rotation2d.fromDegrees(m_turnEncoder.getPosition())); 
    }

}