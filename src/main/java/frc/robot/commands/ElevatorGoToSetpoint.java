// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorGoToSetpoint extends Command {
  private double  m_setpoint;
  private double m_desiredSetpoint;
  private double m_currentPos;
  private Elevator m_elevator;


  public ElevatorGoToSetpoint(double setpoint, Elevator elevator) {
    m_desiredSetpoint = setpoint;
    m_elevator = elevator;
    
    m_currentPos = m_elevator.getPositon();

    m_setpoint = setpoint;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    //m_elevator.refreshSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // m_elevator.setControl(m_setpoint);
   //m_elevator.setGoal(m_desiredSetpoint, 0); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return MathUtil.isNear(m_desiredSetpoint, m_elevator.getPositon(), 1e-5);
  }
}
