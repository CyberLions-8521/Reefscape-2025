// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Kitbot;

public class RobotContainer {
  private CommandXboxController m_controller = new CommandXboxController(0);
  private Kitbot m_kitbot = new Kitbot();
  private Shoot m_shootCommand = new Shoot(m_kitbot, 0.75);
  private Shoot m_reverseShootCommand = new Shoot(m_kitbot, -0.75);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_controller.a().whileTrue(m_shootCommand);
    m_controller.b().whileTrue(m_reverseShootCommand);
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
