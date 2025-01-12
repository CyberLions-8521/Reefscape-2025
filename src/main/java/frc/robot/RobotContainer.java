// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivebase;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final CommandXboxController m_gamepad = new CommandXboxController(0);
  private final Drivebase m_db = new Drivebase();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_db.setDefaultCommand(getDriveCommand(m_gamepad::getLeftY, m_gamepad::getLeftX, m_gamepad::getRightX, true));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  /**
   * Creates a command that runs the drive method from the Drivebase object
   * @param xSpeed  A method reference or lambda that returns the speed of the chassis in the x-direction
   * @param ySpeed  A method reference or lambda that returns the speed of the chassis in the y-direction
   * @param rot     A method reference or lambda that returns the speed of the chassis in the z-rotational direction
   * @param fieldRelative   Whether or not you are driving the robot with speeds relative to the field
   * @return  The drive command
   */
  public Command getDriveCommand(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rot, boolean fieldRelative) {
    return new RunCommand(
      () -> m_db.drive(
        -MathUtil.applyDeadband(xSpeed.get(), ControllerConstants.kDeadband) * DriveConstants.kMaxSpeedMetersPerSecond,
        -MathUtil.applyDeadband(ySpeed.get(), ControllerConstants.kDeadband) * DriveConstants.kMaxSpeedMetersPerSecond,
        -MathUtil.applyDeadband(rot.get(), ControllerConstants.kDeadband) * DriveConstants.kMaxAngularSpeed,
        fieldRelative),
      m_db);
  }
}
