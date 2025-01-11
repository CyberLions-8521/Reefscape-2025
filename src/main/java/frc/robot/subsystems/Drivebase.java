package frc.robot.subsystems;

import frc.robot.Constants.SwerveConstants;
import frc.robot.SwerveModule;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivebase extends SubsystemBase {
    private final SwerveModule m_frontLeft = new SwerveModule(
        kFrontLeftDriveID, kFrontLeftTurnID);
    private final SwerveModule m_frontRight = new SwerveModule(kFrontRightDriveID, kFrontRightTurnID);
    private final SwerveModule m_backLeft = new SwerveModule(kFrontLeftDriveID, kFrontLeftTurnID);
    private final SwerveModule m_backRight = new SwerveModule(kFrontLeftDriveID, kFrontLeftTurnID);


    private final Translation2d m_frontLeftLoc;
    private final Translation2d m_frontRightLoc;
    private final Translation2d m_backLeftLoc;
    private final Translation2d m_backRightLoc;

    public Drivebase() {
        
    }
}
