// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.Configs.MotorConfigs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    m_master.configure(MotorConfigs.SHOOT_MASTER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_slave.configure(MotorConfigs.SHOOT_SLAVE_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter distance", getDistance());
  }

  public void setSpeed(double speed) {
    m_master.set(speed);
  }

  public double getDistance() {
    return ((Math.abs(m_masterEncoder.getPosition()) + Math.abs(m_slaveEncoder.getPosition())) / 2);
  }

  public void resetEncoders() {
    m_masterEncoder.setPosition(0.0);
    m_slaveEncoder.setPosition(0.0);
  }
}
