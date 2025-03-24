// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Subsystems.Swerve;


public class AlignToReef extends Command {
    private final Swerve m_db;

    public AlignToReef(Swerve m_db) {
        this.m_db = m_db;
        this.addRequirements(m_db);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //m_db.drive(0, m_db.alignToReefCalculate(LimelightHelpers.getTX(LimelightConstants.kName)), 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        m_db.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        //return MathUtil.isNear(distance, LimelightHelpers.getTX(LimelightConstants.kName), LimelightConstants.tagXOffsetTolerance) || !LimelightHelpers.getTV(LimelightConstants.kName);
        return true;
    }
}
