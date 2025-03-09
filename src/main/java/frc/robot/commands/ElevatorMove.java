// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorMove extends Command {
  private final Elevator m_elevator;
  private final double m_speed;
 
  public ElevatorMove(Elevator elevator, double speed) {
    m_elevator = elevator;
    m_speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean goingUp   = m_speed > 0 && m_elevator.getPosition() < ElevatorConstants.kMaxHeight;
    boolean goingDown = m_speed < 0 && m_elevator.getPosition() > ElevatorConstants.kMinHeight;
    return !goingUp && !goingDown;  // ! (goingUp || goingDown)
    // return m_elevator.getPosition() >= 6.03;  // 6.03 is experimentally deteremined "max height"
    // return (MathUtil.isNear(6.03, m_elevator.getPosition(), 0.05));
  }
}
