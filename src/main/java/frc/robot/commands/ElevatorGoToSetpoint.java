// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorGoToSetpoint extends Command {
  private double  m_setpoint;
  private Elevator m_elevator;
  private double m_speed;


  public ElevatorGoToSetpoint(double goal, Elevator elevator) {
    m_setpoint = goal; 
    m_elevator = elevator;
    m_setpoint = setpoint;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_elevator.stopMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.goToSetpoint();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_elevator.getPosition - m_setpoint > 0.05){
        true;
    }
    else{ 
        false;
    }
  }
}
