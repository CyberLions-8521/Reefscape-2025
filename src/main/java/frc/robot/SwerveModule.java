
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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveModule {
    private SparkMax m_driveMotor;
    private SparkMax m_turnMotor; 

    private SparkClosedLoopController m_drivePID;
    private SparkClosedLoopController m_turnPID;

    private SparkMaxConfig m_configDrive;
    private SparkMaxConfig m_configTurn;

    private RelativeEncoder m_driveEncoder;
    private RelativeEncoder m_turnEncoder;

    private CANcoder m_CANcoder;

    // for debugging
    private SwerveModuleState m_desiredState = new SwerveModuleState();


    public SwerveModule(int driveMotorPort, int turnMotorPort, int CANCoderPort, double magnetOffset, double absoluteSensorDiscont) {
        m_driveMotor = new SparkMax(driveMotorPort, SparkLowLevel.MotorType.kBrushless);
        m_turnMotor = new SparkMax(turnMotorPort, SparkLowLevel.MotorType.kBrushless);

        m_CANcoder = new CANcoder(CANCoderPort, SwerveConstants.kCANCoderBus);
        m_driveEncoder = m_driveMotor.getEncoder();
        m_turnEncoder = m_turnMotor.getEncoder();
        m_drivePID = m_driveMotor.getClosedLoopController();
        m_turnPID = m_turnMotor.getClosedLoopController();
        m_configDrive = new SparkMaxConfig();
        m_configTurn = new SparkMaxConfig();

        //m_driveMotor.configure(SwerveModuleConfigs.m_configDrive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //m_turnMotor.configure(SwerveModuleConfigs.m_configTurn, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        resetEncoder();

        configMagnets(magnetOffset, absoluteSensorDiscont);
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

    public double getDistance() {
        return m_turnEncoder.getPosition();
    }

    public void turnMotors(double speed, double steer) {
        m_driveMotor.set(speed);
        m_turnMotor.set(steer);
    }

    //for smartdashboard debugging
    public double getDriveDistance() {
        return m_driveEncoder.getPosition(); //rotations
    }

    //for smartdashboard logging purposes
    public double getCANCoderPosition() {
        return m_CANcoder.getAbsolutePosition().getValueAsDouble(); //rotations 
    }

    public void configMagnets(double kCANCoderMagnetOffset, double kCANCoderAbsoluteSensorDiscontinuityPoint) {
        MagnetSensorConfigs m_magnetConfigs = new MagnetSensorConfigs();

        m_magnetConfigs
            .withMagnetOffset(kCANCoderMagnetOffset)
            .withAbsoluteSensorDiscontinuityPoint(kCANCoderAbsoluteSensorDiscontinuityPoint)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
         
        m_CANcoder.getConfigurator().apply(m_magnetConfigs);
        
    }


    public void logData(String motor){
        
        // SmartDashboard.putNumber(motor + "drive position", m_driveEncoder.getPosition());
        // SmartDashboard.putNumber(motor +  "drive velocity", m_driveEncoder.getVelocity());
        SmartDashboard.putNumber(motor +  "turn position", m_turnEncoder.getPosition() % 360 - 180);
        SmartDashboard.putNumber(motor + " CANcoder", m_CANcoder.getAbsolutePosition().getValueAsDouble()*SwerveConstants.kAngleConversion);
        // // SmartDashboard.putNumber(motor + "desired Position ", m_desiredState.angle.getRadians());
        // SmartDashboard.putNumber(motor + "desired Velocity", m_desiredState.speedMetersPerSecond);  
        // SmartDashboard.putNumber(motor + "Ramp Rate", m_turnMotor.configAccessor.getClosedLoopRampRate());  

        //SmartDashboard.putNumber(motor + "turn angle", Math.abs(m_turnEncoder.getPosition()));
        SmartDashboard.putNumber(motor + "desired position", m_desiredState.angle.getDegrees());

            
        
    }

    public SparkMaxConfigAccessor getConfigAccessor() {
        return m_driveMotor.configAccessor;
    }

}
