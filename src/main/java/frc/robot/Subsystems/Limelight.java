package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {

    private final String limelightName = "limelight-twoplus";
    public static final int defaultPipeline = 0;

    public Limelight(int pipeline) {
        setPipelineIndex(pipeline);
    }

    public Limelight() {
        setPipelineIndex(defaultPipeline);
    }

    public double getTX()  { return LimelightHelpers.getTX(limelightName); }
    public double getTY()  { return LimelightHelpers.getTY(limelightName); }
    public double getTA()  { return LimelightHelpers.getTA(limelightName); }
    public boolean getTV() { return LimelightHelpers.getTV(limelightName); }
    public int getTagID()  { return (int)LimelightHelpers.getFiducialID(limelightName); }  

    public void setPipelineIndex(int pipeline){
        LimelightHelpers.setPipelineIndex(limelightName, pipeline);
    }

    public void logData() {
        SmartDashboard.putNumber("target x angle", getTX());
        SmartDashboard.putNumber("target y angle", getTY());
        SmartDashboard.putNumber("target area", getTA());
        SmartDashboard.putBoolean("target valid", getTV());
        SmartDashboard.putNumber("tag id", getTagID());
    }

    @Override
    public void periodic() {
        logData();
    }

}