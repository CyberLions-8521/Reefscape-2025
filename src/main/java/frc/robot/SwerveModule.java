package frc.robot;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    
    

    public SwerveModule() {
        SparkMax TopRightDriveMotor = new SparkMax(SwerveConstants.kTopRightDriveID, SparkLowLevel.MotorType.kBrushless);
        SparkMax TopRightTurnMotor = new SparkMax(SwerveConstants.kTopRightTurnID, SparkLowLevel.MotorType.kBrushless);
        SwerveModuleState m_swerveModuleState = new SwerveModuleState();
    }

}