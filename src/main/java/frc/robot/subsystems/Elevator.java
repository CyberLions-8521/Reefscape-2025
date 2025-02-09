package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Configs.KrakenConfigs;

public class Elevator extends SubsystemBase {
    PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);
    
    private TalonFX m_motor;

    public Elevator(int motorPort) {
        m_motor = new TalonFX(motorPort);
        
        m_motor.getConfigurator().apply(KrakenConfigs.KRAKEN_CONFIGURATION);
    }

public StatusCode setControl(PositionDutyCycle setpoint) {
    return m_motor.setControl(setpoint);
}

public double getPositon() {
    return m_motor.getPosition().getValueAsDouble();
}

public void setSpeed(double speed) {
    m_motor.set(speed);
}
    

}

