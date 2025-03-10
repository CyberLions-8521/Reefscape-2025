// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intake extends Command {
  Shooter m_shooter;
  double m_distance;

  public Intake(Shooter shooter, double distance) {
    m_shooter = shooter;
    m_distance = distance;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_shooter.resetEncoders();
    m_shooter.setSpeed(0.2);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_shooter.setSpeed(0.0);
  }

  @Override
  public boolean isFinished() {
    return (MathUtil.isNear(m_distance, m_shooter.getDistance(), 0.5));
  }
}
