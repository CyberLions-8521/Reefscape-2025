// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ElevatorGo;
import frc.robot.commands.ElevatorGoToSetpoint;
import frc.robot.commands.Intake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private final Elevator m_elevator = new Elevator(10,5);
  private final Shooter m_shooter = new Shooter(0, 0);
  private final CommandXboxController m_controller = new CommandXboxController(0);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //m_controller.a().onTrue(new ElevatorGoToSetpoint(.30, m_elevator));
    m_controller.rightTrigger().whileTrue(new ElevatorGo(m_elevator, .45));
    m_controller.leftTrigger().whileTrue(new ElevatorGo(m_elevator, -.4));
    //m_controller.a().onTrue(m_elevator.resetEncoderCommand());

    m_controller.a().whileTrue(new Intake(m_shooter, .4));
    m_controller.x().whileTrue(new Intake(m_shooter, -.4));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
