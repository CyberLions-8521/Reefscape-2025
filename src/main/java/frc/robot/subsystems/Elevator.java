// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** For REVLib imports, lookup rev java api */
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Elevator extends SubsystemBase {
  // Attributes of the subsystem - hardware
  // Can name your motor whatever you want
  private final SparkMax m_leader = new SparkMax(ElevatorConstants.kLeaderID, MotorType.kBrushless);
  private final SparkMax m_follower = new SparkMax(ElevatorConstants.kFollowerID, MotorType.kBrushless);
  
  /** Creates a new Elevator. */
  public Elevator() {
    // should probably configure the motors in here
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // An example method for the Elevator class
  public void setSpeed(double speed) {
    m_leader.set(speed);
  }
}
