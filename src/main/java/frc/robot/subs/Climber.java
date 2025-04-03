// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs.ClimberConfigs;
import frc.robot.Configs.ElevatorConfigs;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private TalonFX m_motor; //using a Kraken

  public Climber(int motorID) {
    m_motor = new TalonFX(motorID);
    
    
    m_motor.getConfigurator().apply(ClimberConfigs.kKrakenConfig);
  }


public Command moveClimberCommand(double speed){
  return this.run(() -> this.m_motor.set(speed));
}


  /*The isFinished will prevent the motors from going past what is possible becaues of the ! .
   * The motor will stop when the position is NOT greater than the home position AND negative
   * The motor will stop when the position is NOT less than the maxEntend point AND positive
   * Once the motor goes past these points, it will not allow for further extenstion / retraction
   */
  private boolean isFinished() {
               //is negative                                    //greater or equal to than the negative limit
    return (!(((m_motor.getVelocity().getValueAsDouble() < 0.0) && (m_motor.getVelocity().getValueAsDouble() >= ClimberConstants.kHomePosition)) || 
              (m_motor.getVelocity().getValueAsDouble() > 0.0) && (m_motor.getVelocity().getValueAsDouble() <= ClimberConstants.kMaxExtenstion)));
               //is positive                                    //less than or equal to the max limit
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}