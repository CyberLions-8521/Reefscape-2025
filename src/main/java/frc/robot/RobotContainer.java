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
import frc.robot.commands.ElevatorGo;
import frc.robot.commands.ElevatorGoToSetpoint;
import frc.robot.commands.Intake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.TestTickles;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveDrivebaseConstants;

public class RobotContainer {
  private final Elevator m_elevator = new Elevator(10,5);
  private final Shooter m_shooter = new Shooter(0, 0);
  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final Swerve m_db = new Swerve();

  public RobotContainer() {
    configureBindings();
  }
 
  private void configureBindings() {
    //m_controller.a().onTrue(new ElevatorGoToSetpoint(.30, m_elevator));
    m_controller.rightTrigger().whileTrue(new ElevatorGo(m_elevator, .45));
    m_controller.leftTrigger().whileTrue(new ElevatorGo(m_elevator, -.4));
    //m_controller.a().onTrue(m_elevator.resetEncoderCommand());

    m_controller.a().whileTrue(new Intake(m_shooter, .4));
    m_controller.x().whileTrue(new Intake(m_shooter, .6));

    // m_XboxController.a().onTrue(new InstantCommand(m_db::setSpeed1, m_db));
    // m_XboxController.x().onTrue(new InstantCommand(m_db::setSpeed2, m_db));
    // m_XboxController.y().onTrue(new InstantCommand(m_db::stopMotors, m_db));
    m_db.setDefaultCommand(getDriveCommand(m_XboxController::getLeftY, m_XboxController::getLeftX, m_XboxController::getRightX, m_XboxController.getHID()::getRightBumperButton));

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
