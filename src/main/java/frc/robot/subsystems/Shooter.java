// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ShooterConfigs;
import frc.robot.Constants.ElevatorConstants;

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

  //SHOOTER
  public Command getIntakeComamand(double distance) {
        return new FunctionalCommand(
            () -> {
              this.resetEncoders();
              this.setSpeed(0.2);
            },
            () -> {},
            interrupted -> this.setSpeed(0.0),
            () -> (MathUtil.isNear(distance, this.getDistance(), 0.5)),
            this);
        }



  public Command getShootCommand(double speed) {
        return new FunctionalCommand(
            () -> {},
            () -> {
              this.setSpeed(speed);
            },
            interrupted -> this.setSpeed(0.0),
            () -> false,
            this);
    }


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