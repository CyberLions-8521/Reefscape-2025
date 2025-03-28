// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.Algae;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LiftAlgae extends Command {
  /** Creates a new LiftAlgae. */
  Elevator m_elev;
  Algae m_algae;

  public LiftAlgae(Algae algae, Elevator elevator) {
    m_elev = elevator;
    m_algae = algae;
    
    addRequirements(algae);
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elev.getSetpointCommand(ElevatorConstants.kL1Setpoint)
          .andThen(new WaitCommand(.2))
          .andThen(m_algae.algaeUp())
          .andThen(new WaitCommand(.2))
          .andThen(m_elev.getSetpointCommand(ElevatorConstants.kBaseSetpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
