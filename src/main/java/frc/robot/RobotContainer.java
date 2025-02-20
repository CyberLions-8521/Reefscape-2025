// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ElevatorGo;
import frc.robot.Commands.ElevatorGoToSetpoint;
import frc.robot.Commands.Intake;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveDrivebaseConstants;

public class RobotContainer {
  private final Elevator m_elevator = new Elevator(10,5);
  private final Shooter m_shooter = new Shooter(0, 0);
  private final CommandXboxController m_driveController = new CommandXboxController(0);
  private final CommandXboxController m_commandController = new CommandXboxController(0);
  private final Swerve m_db = new Swerve();

  public RobotContainer() {
    configureBindings();
  }
 
  private void configureBindings() {

    m_driveController.rightTrigger().onTrue(m_db.getResetEncodersCommand());
    m_driveController.leftTrigger().onTrue(m_db.getResetGyroCommand());
    m_db.setDefaultCommand(m_db.getDriveCommand(m_driveController::getLeftY, m_driveController::getLeftX, m_driveController::getRightX, m_driveController.getHID()::getRightBumperButton));

    m_commandController.leftBumper().onTrue(new ElevatorGoToSetpoint(ElevatorConstants.kRestingPosition, m_elevator));
    m_commandController.a().onTrue(new ElevatorGoToSetpoint(ElevatorConstants.kL1Position, m_elevator));
    m_commandController.b().onTrue(new ElevatorGoToSetpoint(ElevatorConstants.kL2Position, m_elevator));
    m_commandController.x().onTrue(new ElevatorGoToSetpoint(ElevatorConstants.kL3Position, m_elevator));
    m_commandController.y().onTrue(new ElevatorGoToSetpoint(ElevatorConstants.kL4Position, m_elevator));

  }

  public Command getAutonomousCommand() {
    return Commands.print(
      
    "No autonomous command configured");
  }

  
}
