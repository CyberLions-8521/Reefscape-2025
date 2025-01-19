// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ElevatorGoToSetpoint;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
  private final Elevator m_elevator = new Elevator(0);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_controller.a().onTrue(new ElevatorGoToSetpoint(m_elevator, 0.30));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
