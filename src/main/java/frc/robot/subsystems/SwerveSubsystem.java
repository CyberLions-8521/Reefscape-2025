package frc.robot.subsystems;

import frc.robot.SwerveModule;

import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {

    SparkMaxConfig driveConfig;
    SparkMaxConfig turnConfig;
    
    SwerveModule frontLeftModule;
    SwerveModule frontRightModule;
    SwerveModule backLeftModule;
    SwerveModule backRightModule;

    public SwerveSubsystem() {
        //4 swerve modules (ports are placeholders)
        frontLeftModule = new SwerveModule(0,0);
        frontRightModule = new SwerveModule(0, 0);
        backLeftModule = new SwerveModule(0, 0);
        backRightModule = new SwerveModule(0, 0);

        

        //(+,+) = topLeft

    }

    public void configurations(){
        driveConfig.encoder.positionConversionFactor(1);
    }
}
