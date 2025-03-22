// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve;

public class DriveToDistance extends Command {
  private final Swerve m_db;
  private final double m_distance;

  public DriveToDistance(Swerve db, double distance) {
    m_db = db;
    m_distance = distance;
    addRequirements(db);
  }

  @Override
  public void initialize() {
    m_db.resetEncoders();
  }

  @Override
  public void execute() {
    m_db.drive(1.0, 0, 0,true);
  }

  @Override
  public void end(boolean interrupted) {
    m_db.drive(0.0, 0.0, 0.0, false);
  }

  @Override
  public boolean isFinished() {
    return (MathUtil.isNear(m_distance, m_db.getStraightDistance(), 0.1));
  }
}