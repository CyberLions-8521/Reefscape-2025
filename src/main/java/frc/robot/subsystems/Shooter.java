// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.Configs.MotorConfigs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  SparkMax m_master;
  SparkMax m_slave;
  
  /** Creates a new Shooter. */
  public Shooter(int masterID, int slaveID) {
    m_master = new SparkMax(masterID, MotorType.kBrushless);
    m_slave = new SparkMax(slaveID, MotorType.kBrushless);

    m_master.configure(MotorConfigs.SHOOT_MASTER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_slave.configure(MotorConfigs.SHOOT_SLAVE_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    m_master.set(speed);
  }
}
