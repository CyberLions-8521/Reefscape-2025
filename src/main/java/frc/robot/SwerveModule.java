package frc.robot;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
        //Variables
        SparkMax driveMotor;
        SparkMax turnMotor;
        SparkClosedLoopController drivePID;

        
        
        SwerveModuleState currentState;
        SwerveModuleState desireState;

        
        PIDController drivePIDController;
        PIDController turnPIDController;

    public SwerveModule(int driveMotorPort, int turnMotorPort) {
        System.out.println("SwerveModule constructor");
        driveMotor = new SparkMax(turnMotorPort, MotorType.kBrushless);
        turnMotor = new SparkMax(turnMotorPort, MotorType.kBrushless);
        drivePID = driveMotor.getClosedLoopController();

        //create PID Controllers
        drivePIDController = new PIDController(SwerveConstants.drivePIDControllerP, SwerveConstants.drivePIDControllerI, SwerveConstants.drivePIDControllerD);
        turnPIDController = new PIDController(SwerveConstants.turnPIDControllerP, SwerveConstants.turnPIDControllerI, SwerveConstants.turnPIDControllerD);

        currentState = new SwerveModuleState();

        /*
config
    .inverted(true)
    .idleMode(IdleMode.kBrake);
config.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);
config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1.0, 0.0, 0.0);
    
max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); */
        
    }
    


    public SwerveModuleState getState(){ //returns smodule's speed & angle
        return currentState;
    }

    public void setDesiredState(SwerveModuleState desireState){
        drivePID.setReference(desireState.speedMetersPerSecond, ControlType.kVelocity);
    }

    public void periodic(){
         
    }

    public void configure(SparkMax motor){

    }


}