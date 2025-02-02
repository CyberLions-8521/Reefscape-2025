package frc.robot;

import frc.robot.Configs.SwerveModuleConfig;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private final SparkMax m_driveMotor;
    private final SparkMax m_turnMotor;
    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_turnEncoder;
    private final SparkClosedLoopController m_drivePID;
    private final SparkClosedLoopController m_turnPID;
    private final CANcoder m_canCoder;

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

    public void reCalibrateTurnEncoder() {
        m_turnEncoder.setPosition(m_canCoder.getAbsolutePosition().getValueAsDouble());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentRotation = new Rotation2d(m_turnEncoder.getPosition());
        // Optimize to allow module to make the smallest turn to arrive at the rotational setpoint
        // e.g., it will turn through 0 degrees when going from 350 degrees to 10 degrees
        desiredState.optimize(currentRotation);

        // Taken from WPILib example; comment out if it doesn't work
        desiredState.cosineScale(currentRotation);

        m_drivePID.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
        m_turnPID.setReference(desiredState.angle.getRotations(), ControlType.kPosition);
    }

}