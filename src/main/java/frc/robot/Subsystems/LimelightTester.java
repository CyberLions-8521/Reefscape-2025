package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightTester extends SubsystemBase {

    private final String limelightName = "limelight-twoplus";
    public static final int defaultPipeline = 0;



    public LimelightTester(int pipeline) {
        setPipelineIndex(pipeline);
    }

    public LimelightTester() {
        setPipelineIndex(defaultPipeline);
    }

    public double getTX()  { return LimelightHelpers.getTX(limelightName); } // horizontal offset in degrees
    public double getTY()  { return LimelightHelpers.getTY(limelightName); } // vertical offset in degrees
    public double getTA()  { return LimelightHelpers.getTA(limelightName); } // area of the thing being detected (e.g. apriltag)
    public boolean getTV() { return LimelightHelpers.getTV(limelightName); } // valid target?
    public int getTagID()  { return (int)LimelightHelpers.getFiducialID(limelightName); }  // tag id

    // pipeline: how the limelight processes images (like color vs black and white)
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