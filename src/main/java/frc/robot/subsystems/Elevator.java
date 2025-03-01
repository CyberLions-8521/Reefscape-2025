package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.MotorConfigs;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);
    SparkClosedLoopController closedLoopController;
    
    //private TalonFX m_motor;
    private SparkMax m_motorMaster;
    private SparkMax m_motorSlave;

    private RelativeEncoder m_encoder;

    /*private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints( 0,0 ));
    *private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    *private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(getPositon(),0);
*/
    

    public Elevator(int masterMotorPort, int slaveMotorPort) {
        //m_motor = new TalonFX(motorPort);
        m_motorMaster = new SparkMax(masterMotorPort, MotorType.kBrushless);
        m_motorSlave = new SparkMax(slaveMotorPort, MotorType.kBrushless);

        m_motorMaster.configure(MotorConfigs.ELEV_MASTER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_motorSlave.configure(MotorConfigs.ELEV_SLAVE_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder = m_motorMaster.getEncoder();

        closedLoopController = m_motorMaster.getClosedLoopController();
        //m_motor.getConfigurator().apply(KrakenConfigs.KRAKEN_CONFIGURATION);
    }

    public double getPositon() { //INCHES
        return m_encoder.getPosition();
    }
    
    public SparkClosedLoopController getController() {
        return closedLoopController;
    }
    

// public StatusCode setControl(PositionDutyCycle setpoint) {
//     return m_motorMaster.setControl(setpoint);
// }
/* 
private double calculateFeedForward(TrapezoidProfile.State state){
    //kS (static friction): kG (gravity): kV (velocity)
    return (ElevatorConstants.kS * Math.signum(state.velocity)) 
        + ElevatorConstants.kG + (ElevatorConstants.kV * state.velocity);
}


public void updateSetpoint(){
    m_setpoint = new TrapezoidProfile.State(getPositon(), m_encoder.getVelocity());
}

public boolean atSetpoint(){
    return MathUtil.isNear(m_goal.position, getPositon(), 0.5);
}



public void goToSetpoint(double desiredPosition, double desiredVelocity){
    m_goal =  new TrapezoidProfile.State(desiredPosition, desiredVelocity);
    m_setpoint = m_profile.calculate(0.02, m_setpoint, m_goal); //value needs to be set
    double output = MathUtil.clamp(m_setpoint.position, 0, 26); //value needs to be set
    closedLoopController.setReference(output, ControlType.kPosition);

}*/


public void setSpeed(double speed) {
    m_motorMaster.set(speed);
}

//for testing purposes
public Command resetEncoderCommand() {
    return this.run(() -> resetEncoder());
}

//for testing purposes
public void resetEncoder() {
    m_encoder.setPosition(0.00);
}

public void logData() {
    SmartDashboard.putNumber("Elevator Position", m_encoder.getPosition());
}

@Override
public void periodic() {
    logData();
}

}