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
    private double m_offset=0;

    public AutoAlignToReef(Swerve m_db, double m_setpoint) {
        this.m_db = m_db;
        this.m_setpoint = m_setpoint; 
        this.addRequirements(m_db);
    }

    @Override
    public void initialize() {
        // m_db.setEncoderDistance(m_db.calculateDistanceFromAprilTag());
        m_offset = m_db.calculateDistanceFromAprilTag();
        m_db.resetEncoders();
        m_db.setReefAlignSetpoint(m_setpoint);
    }

    @Override
    public void execute() {
        //m_setpoint - m_db.calculateDistanceFromAprilTag() is the distance to the setpoint/reef
        double currentDistance = m_db.getStraightDistance();
        if (m_setpoint < 0){
            currentDistance*=-1;
        }
        m_db.drive(0,m_db.getAlignPID().calculate(currentDistance+m_offset), 0, false);
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
