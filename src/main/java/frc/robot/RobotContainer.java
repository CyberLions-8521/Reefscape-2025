// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ElevatorGo;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
  Elevator m_elevator = new Elevator(0, 1);
  CommandXboxController m_controller = new CommandXboxController(0);
  ElevatorGo liftElevatorCmd = new ElevatorGo(m_elevator);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_controller.a().whileTrue(liftElevatorCmd);

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
