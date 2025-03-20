// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Subsystems.Swerve;


public class AlignToReef extends Command {
    
    private final Swerve m_db;
    private boolean isRight;
    String limelightName = LimelightConstants.limelightName;

    private int[] teamReefTags;

    public AlignToReef(String teamColor, Swerve m_db, boolean isRight) {
        this.m_db = m_db;
        this.addRequirements(m_db);

        if (teamColor.equals("blue")) {
            teamReefTags = LimelightConstants.blueTags;
        } else if (teamColor.equals("red")) {
            teamReefTags = LimelightConstants.redTags;
        }
        
        this.isRight = isRight;
    }

    public boolean tagListIncludes(int[] tagList, int target) {
        for (int tagId : tagList) {
            if (tagId == target) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        boolean targetValid = LimelightHelpers.getTV(limelightName);
        int tagId = (int) LimelightHelpers.getFiducialID(limelightName);

        if (targetValid && tagListIncludes(teamReefTags, tagId)) {
            m_db.drive(0, m_db.m_alignPID.calculate(LimelightHelpers.getTX(LimelightConstants.limelightName), 0.0), 0, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_db.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        boolean targetValid = LimelightHelpers.getTV(limelightName);
        int tagId = (int) LimelightHelpers.getFiducialID(limelightName);

        if (targetValid && tagListIncludes(teamReefTags, tagId)) {
            double tagXOffset = LimelightHelpers.getTX(limelightName);
            double tagXOffsetTolerance = 5.0;
            return Math.abs(tagXOffset) <= tagXOffsetTolerance;
        }

        return false;
    }
}
