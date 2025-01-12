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
import frc.robot.Subsystems.Swerve;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveDrivebaseConstants;


public class RobotContainer {
  private final CommandXboxController m_XboxController = new CommandXboxController(0);
  private final Swerve m_db = new Swerve();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_db.setDefaultCommand(getDriveCommand(m_XboxController::getLeftY, m_XboxController::getLeftX, m_XboxController::getRightX, true));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

    public Command getDriveCommand(Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> omega, boolean fieldRelative) {
    return new RunCommand(
      () -> m_db.drive(
        -MathUtil.applyDeadband(vx.get(), ControllerConstants.kDeadband) * SwerveDrivebaseConstants.kMaxMetersPerSecond,
        -MathUtil.applyDeadband(vy.get(), ControllerConstants.kDeadband) * SwerveDrivebaseConstants.kMaxMetersPerSecond,
        -MathUtil.applyDeadband(omega.get(), ControllerConstants.kDeadband) * SwerveDrivebaseConstants.kMaxAngularSpeed,
        fieldRelative),
      m_db);
    
  }
}
