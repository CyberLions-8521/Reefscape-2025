package frc.robot;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;


public class SwerveModule {
        //Variables
        private final SparkMax m_driveMotor;
        private final SparkMax m_turnMotor;

        private final RelativeEncoder m_driveEncoder;
        private final RelativeEncoder m_turnEncoder;

        private final SparkClosedLoopController m_driveClosedLoopController;
        private final SparkClosedLoopController m_turnClosedLoopController;

        private double m_chassisAngularOffset = 0;
        private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

        
        public SwerveModule(int driveCanID, int turnCanID, double chassisAngularOffset){
            m_driveMotor = new SparkMax(driveCanID, MotorType.kBrushless);
            m_turnMotor = new SparkMax(turnCanID, MotorType.kBrushless);

            m_driveEncoder = m_driveMotor.getEncoder();
            m_turnEncoder = m_turnMotor.getEncoder();

            m_driveClosedLoopController = m_driveMotor.getClosedLoopController();
            m_turnClosedLoopController = m_turnMotor.getClosedLoopController();


            m_driveMotor.configure(Configs.SwerveModule.driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_turnMotor.configure(Configs.SwerveModule.turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            m_chassisAngularOffset = chassisAngularOffset;
            m_desiredState.angle = new Rotation2d(m_turnEncoder.getPosition());
            m_driveEncoder.setPosition(0);
        }

        public void setDesiredState(SwerveModuleState targetState){
            Rotation2d currentRotation = Rotation2d.fromDegrees(m_turnEncoder.getPosition());
            targetState.optimize(currentRotation);

            m_driveClosedLoopController.setReference(targetState.speedMetersPerSecond, ControlType.kVelocity);
            m_driveClosedLoopController.setReference(targetState.angle.getDegrees(), ControlType.kPosition);

        }

        public SwerveModulePosition getPosition(){
            return new SwerveModulePosition(
                m_driveEncoder.getPosition(), new Rotation2d(m_turnEncoder.getPosition() - m_chassisAngularOffset));
            
        }

        public SwerveModuleState getState(){
            return new SwerveModuleState(m_driveEncoder.getVelocity(),
            new Rotation2d(m_turnEncoder.getPosition() - m_chassisAngularOffset));        }

        public void resetEncoders(){
            m_driveEncoder.setPosition(0);
        }






}