package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.ejml.dense.block.MatrixOps_DDRB;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs.MotorConfigs;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class Elevator extends SubsystemBase {
    PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);
    SparkClosedLoopController closedLoopController;
    
    //private TalonFX m_motor;
    private SparkMax m_motorMaster;
    private SparkMax m_motorSlave;

    private RelativeEncoder m_encoder;
    

    public Elevator(int masterMotorPort, int slaveMotorPort) {
        //m_motor = new TalonFX(motorPort);
        m_motorMaster = new SparkMax(masterMotorPort, MotorType.kBrushless);
        m_motorSlave = new SparkMax(slaveMotorPort, MotorType.kBrushless);

        m_motorMaster.configure(MotorConfigs.ELEV_MASTER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_motorSlave.configure(MotorConfigs.SPARK_CONFIGURATION, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder = m_motorMaster.getEncoder();

        closedLoopController = m_motorMaster.getClosedLoopController();
        //m_motor.getConfigurator().apply(KrakenConfigs.KRAKEN_CONFIGURATION);
    }

// public StatusCode setControl(PositionDutyCycle setpoint) {
//     return m_motorMaster.setControl(setpoint);
// }

public double getPositon() {
    return m_encoder.getPosition();
}

public SparkClosedLoopController getController() {
    return closedLoopController;
}

public void setSpeed(double speed) {
    m_motorMaster.set(speed);
}

public Command resetEncoderCommand() {
    return this.run(() -> resetEncoder());
}

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

