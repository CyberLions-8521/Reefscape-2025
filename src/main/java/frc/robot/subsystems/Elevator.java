package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Configs.MotorConfigs;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private SparkMax m_motorMaster;
    private SparkMax m_motorSlave;
    private RelativeEncoder m_encoder;
    private SparkClosedLoopController m_pidController;
    private final ElevatorFeedforward m_elevFF;

    public Elevator(int masterMotorPort, int slaveMotorPort) {
        m_motorMaster = new SparkMax(masterMotorPort, MotorType.kBrushless);
        m_motorSlave  = new SparkMax(slaveMotorPort , MotorType.kBrushless);
        m_encoder = m_motorMaster.getEncoder();
        m_pidController = m_motorMaster.getClosedLoopController();
        m_elevFF = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

        m_motorMaster.configure(MotorConfigs.ELEV_MASTER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motorSlave.configure(MotorConfigs.ELEV_SLAVE_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetEncoder();
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    // Velocity of m_elevFF can be changed if using motion profiling.  Otherwise, set to 0 for anti gravity.
    public REVLibError setReference(double value, ControlType ctrl) {   // for use in external command files
        return m_pidController.setReference(value, ctrl, ClosedLoopSlot.kSlot0, m_elevFF.calculate(0));
    }

    public REVLibError applyAntiGravityFF() {
        return m_pidController.setReference(m_elevFF.calculate(0), ControlType.kVoltage);
    }

    /** Command that applies voltage (calculated via kG) to counteract gravity.
     * Used to set the default command of the elevator when binding buttons that
     * allow for manual movement of the elevator.  Note that if the elevator is
     * going to setpoints, then this is not needed, as the GoToSetpoint commands
     * will never end, and will apply the anti gravity FF via arbFF.
     * @return  REVLibError.kOk if successful
     */
    public Command applyAntiGravFFCommand() {
        return this.run(this::applyAntiGravityFF);
    }

    public void setSpeed(double speed) {
        m_motorMaster.set(speed);
    }

    //for testing purposes
    public Command resetEncoderCommand() {
        return this.run(() -> resetEncoder());
    }

    //for testing purposes
    public void resetEncoder() {
        m_encoder.setPosition(0.0);
    }

    public void logData() {
        SmartDashboard.putNumber("Elevator Position", m_encoder.getPosition());

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