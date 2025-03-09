// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorGoToL4 extends Command {
  private double  m_setpoint;
  private double m_desiredSetpoint;
  private double m_currentPos;
  private Elevator m_elevator;
  private double m_speed;


  public ElevatorGoToL4(double setpoint, Elevator elevator, double speed) {
    m_desiredSetpoint = setpoint;
    m_elevator = elevator;
    m_speed = speed;
    
    m_currentPos = m_elevator.getPositon();

    m_setpoint = setpoint;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    //m_elevator.updateSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setSpeed(m_speed);
   // m_elevator.setControl(m_setpoint);
   //m_elevator.setGoal(m_desiredSetpoint, 0); 
   
   
   //m_elevator.goToSetpoint(m_setpoint, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setSpeed(0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return MathUtil.isNear(m_desiredSetpoint, m_elevator.getPositon(), 1e-5);
    //return m_elevator.atSetpoint();
    
      return (MathUtil.isNear(6.03, m_elevator.getPositon(), 0.05));
    
    
  }
}
