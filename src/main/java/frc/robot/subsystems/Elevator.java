// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private SparkMax masterMotor; 
  private SparkMax slaveMotor;
  private SparkMaxConfig slaveConfig = new SparkMaxConfig();
  private SparkMaxConfig masterConfig = new SparkMaxConfig();

  public Elevator(int masterId, int slaveId) {
    masterMotor = new SparkMax(masterId, MotorType.kBrushless);
    slaveMotor = new SparkMax(slaveId, MotorType.kBrushless);
    
    configMotors(masterId);

    masterMotor.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    slaveMotor.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

// these are methods & functions 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

// dot means accessing function

  public void setSpeed(double speed) {
    masterMotor.set(speed);
    slaveMotor.set(speed);
  }

  public void configMotors(int masterId) {
    masterConfig.inverted(true);

    slaveConfig.follow(masterId, true);
  }
}
