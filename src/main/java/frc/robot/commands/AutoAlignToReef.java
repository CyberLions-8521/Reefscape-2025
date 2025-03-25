// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Subsystems.Swerve;


public class AutoAlignToReef extends Command {
    private final Swerve m_db;
    private final double m_setpoint;

    public AutoAlignToReef(Swerve m_db, double m_setpoint) {
        this.m_db = m_db;
        this.m_setpoint = m_setpoint; 
        this.addRequirements(m_db);
    }

    @Override
    public void initialize() {
        m_db.setEncoderDistance(m_db.calculateDistanceFromAprilTag());
        m_db.setReefAlignSetpoint(m_setpoint);
    }

    @Override
    public void execute() {
        //m_setpoint - m_db.calculateDistanceFromAprilTag() is the distance to the setpoint/reef
        m_db.drive(m_db.getAlignPID().calculate(m_setpoint-m_db.calculateDistanceFromAprilTag()), 0, 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        //stop drive
        m_db.drive(0,0,0, false);
    }

    @Override
    public boolean isFinished() {
        //stop robot
        return MathUtil.isNear(m_setpoint, m_setpoint-m_db.calculateDistanceFromAprilTag(), LimelightConstants.kDistanceToReefThreshold) || !LimelightHelpers.getTV(LimelightConstants.kName);
    }
}
