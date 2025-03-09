// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorGoToSetpoint extends Command {
  private final double m_setpoint;
  private final Elevator m_elevator;

  public ElevatorGoToSetpoint(double setpoint, Elevator elevator) {
    m_setpoint = MathUtil.clamp(setpoint, ElevatorConstants.kMinHeight, ElevatorConstants.kMaxHeight);
    m_elevator = elevator;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setReference(m_setpoint, ControlType.kPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  { m_elevator.setSpeed(0); }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  { return false; }
}
