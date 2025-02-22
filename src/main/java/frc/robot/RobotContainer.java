// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ElevatorGo;
import frc.robot.Commands.Intake;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.OperaterConstants;
import frc.robot.Constants.SwerveDrivebaseConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;

public class RobotContainer {
  private final Elevator m_elevator = new Elevator(ElevatorConstants.kMasterID, ElevatorConstants.kSlaveID);
  private final Shooter m_shooter = new Shooter(ShooterConstants.kMasterID, ShooterConstants.kSlaveID);
  private final CommandXboxController m_driveController = new CommandXboxController(OperaterConstants.kDriveControllerPort);
  private final CommandXboxController m_commandController = new CommandXboxController(OperaterConstants.kCommandControllerPort);
  private final Swerve m_db = new Swerve();

  public RobotContainer() {
    configureBindings();
  }
 
  private void configureBindings() {
    //m_controller.a().onTrue(new ElevatorGoToSetpoint(.30, m_elevator));
    m_commandController.rightTrigger().whileTrue(new ElevatorGo(m_elevator, .45));
    m_commandController.leftTrigger().whileTrue(new ElevatorGo(m_elevator, -.4));
    //m_controller.a().onTrue(m_elevator.resetEncoderCommand());

    m_commandController.a().whileTrue(new Intake(m_shooter, .4));
    m_commandController.x().whileTrue(new Intake(m_shooter, .6));

    

    // m_XboxController.a().onTrue(new InstantCommand(m_db::setSpeed1, m_db));
    // m_XboxController.x().onTrue(new InstantCommand(m_db::setSpeed2, m_db));
    // m_XboxController.y().onTrue(new InstantCommand(m_db::stopMotors, m_db));
    m_driveController.b().onTrue(m_db.resetEncodersCommand());
    m_driveController.a().onTrue(m_db.resetGyroCommand());
    m_db.setDefaultCommand(getDriveCommand(m_driveController::getLeftY, m_driveController::getLeftX, m_driveController::getRightX, m_driveController.getHID()::getRightBumperButton));
    
  }

  public Command getAutonomousCommand() {
    return Commands.print(
      
    "No autonomous command configured");
  }

    
  public Command getDriveCommand(Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> omega, Supplier<Boolean> fieldRelative) {
    return new RunCommand(
      () -> m_db.drive(
        -MathUtil.applyDeadband(vx.get(), ControllerConstants.kDeadband) * SwerveDrivebaseConstants.kMaxMetersPerSecond,
        -MathUtil.applyDeadband(vy.get(), ControllerConstants.kDeadband) * SwerveDrivebaseConstants.kMaxMetersPerSecond,
        -MathUtil.applyDeadband(omega.get(), ControllerConstants.kDeadband) * SwerveDrivebaseConstants.kMaxAngularSpeed,
        !fieldRelative.get()),
      m_db);
  }
  
}
