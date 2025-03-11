package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {
    
    public Limelight(int pipeline) {
        
    }

    public void logData() {
        SmartDashboard.putNumber("t x",  LimelightHelpers.getTX(""));
        SmartDashboard.putNumber("t y", LimelightHelpers.getTY(""));
        SmartDashboard.putNumber("t area", LimelightHelpers.getTA(""));
        SmartDashboard.putBoolean("t valid", LimelightHelpers.getTV(""));
        SmartDashboard.putNumber("tag id", LimelightHelpers.getFiducialID(""));
    }

    @Override
    public void periodic() {
        logData();
    }

}