// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ShooterConfigs;

public class Shooter extends SubsystemBase {
  SparkMax m_master;
  SparkMax m_slave;

  RelativeEncoder m_masterEncoder;
  RelativeEncoder m_slaveEncoder;
  
  /** Creates a new Shooter. */
  public Shooter(int masterID, int slaveID) {
    m_master = new SparkMax(masterID, MotorType.kBrushless);
    m_slave = new SparkMax(slaveID, MotorType.kBrushless);

    m_masterEncoder = m_master.getEncoder();
    m_slaveEncoder = m_slave.getEncoder();
    m_master.configure(ShooterConfigs.kMasterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_slave.configure(ShooterConfigs.kSlaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //DATA LOGGING
  private void logData(){
      SmartDashboard.putNumber("Shooter Position", getDistance());
      SmartDashboard.putNumber("Shooter Velocity", getSpeed());
  }

  //SHOOTER COMMANDS
  
  public void setSpeed(double speed) {
    m_master.set(speed);
  }

  public double getSpeed() {
    return m_master.get();
  }

  public double getDistance() {
    return ((Math.abs(m_masterEncoder.getPosition()) + Math.abs(m_slaveEncoder.getPosition())) / 2);
  }

  public void resetEncoders() {
    m_masterEncoder.setPosition(0.0);
    m_slaveEncoder.setPosition(0.0);
  }

  @Override
  public void periodic() {
    logData();
  }
}