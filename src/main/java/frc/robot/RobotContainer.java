// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivebase;
import java.util.function.Supplier;

public class RobotContainer {
  private final Drivebase m_db = new Drivebase();
  private final CommandXboxController m_Controller = new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_db.setDefaultCommand(getDriveCommand(m_Controller::getLeftY, m_Controller::getLeftX, m_Controller::getRightX, true));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Command getDriveCommand(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rot, boolean fieldRelative){
    return new RunCommand(
      () ->   m_db.drive(
        -MathUtil.applyDeadband(xSpeed.get(), Constants.ControllerConstants.kDeadBand) * Constants.DriveConstants.kMaxSpeedMetersPerSecond,
        -MathUtil.applyDeadband(ySpeed.get(), Constants.ControllerConstants.kDeadBand) * Constants.DriveConstants.kMaxSpeedMetersPerSecond,
        -MathUtil.applyDeadband(rot.get(),Constants.ControllerConstants.kDeadBand) * Constants.DriveConstants.kMaxAngularSpeed,
        fieldRelative
    ),
    m_db);
  }
}
