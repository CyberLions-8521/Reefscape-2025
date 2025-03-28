// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveDrivebaseConstants;
import frc.robot.Subsystems.Swerve;


public class AutoAlignToReefLeft extends Command {
    private final Swerve m_db;
    private final double m_setpoint;
    private Double m_offset;
    //private double currentDistance = 0;

    public AutoAlignToReefLeft(Swerve m_db, double m_setpoint) {
        this.m_db = m_db;
        this.m_setpoint = m_setpoint; 
        this.addRequirements(m_db);
    }

    @Override
    public void initialize() {
        //m_db.setEncoderDistance(m_db.calculateDistanceFromAprilTag());
        m_offset = m_db.calculateDistanceFromAprilTag();
        m_db.resetEncoders();
        m_db.setReefAlignSetpoint(m_setpoint);
    }

    @Override
    public void execute() {
        //m_setpoint - m_db.calculateDistanceFromAprilTag() is the distance to the setpoint/reef
        // currentDistance = m_db.getStraightDistance();
        //m_db.drive(0, m_db.getAlignPID().calculate(currentDistance+m_offset), 0, false);
        m_db.drive(0,SwerveDrivebaseConstants.kAutoAlignSpeed,0,false);
    }

    @Override
    public void end(boolean interrupted) {
        //stop drive
        m_db.drive(0,0,0, false);
    }

    @Override
    public boolean isFinished() {
        //stop robot
        //return false;
        return m_offset == null || m_db.getStraightDistance()+m_offset.doubleValue() >= m_setpoint;
        // return MathUtil.isNear(m_setpoint, currentDistance+m_db.calculateDistanceFromAprilTag(), LimelightConstants.kDistanceToReefThreshold) || !LimelightHelpers.getTV(LimelightConstants.kName);
    }
}
