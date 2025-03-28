package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ElevatorConfigs;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private SparkMax m_motorMaster;
    private SparkMax m_motorSlave;
    private RelativeEncoder m_encoder;
    private SparkClosedLoopController m_pidController;
    private ElevatorFeedforward m_feedForward;
    private final TrapezoidProfile m_profile;
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;
    
    public Elevator(int masterMotorPort, int slaveMotorPort) {
        m_motorMaster = new SparkMax(masterMotorPort, MotorType.kBrushless);
        m_motorSlave  = new SparkMax(slaveMotorPort , MotorType.kBrushless);
        m_encoder = m_motorMaster.getEncoder();
        m_pidController = m_motorMaster.getClosedLoopController();
        m_feedForward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);
        m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration));
        m_goal = new TrapezoidProfile.State();
        m_setpoint = new TrapezoidProfile.State(getPosition(),0);
        m_motorMaster.configure(ElevatorConfigs.kMasterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motorSlave.configure(ElevatorConfigs.kSlaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        resetEncoder();
        putPIDSmartDashboard();
    }

    //DATA LOGGING
    private void putPIDSmartDashboard() {
        SmartDashboard.putNumber("ElevP", m_motorMaster.configAccessor.closedLoop.getP());
        SmartDashboard.putNumber("ElevI", m_motorMaster.configAccessor.closedLoop.getI());
        SmartDashboard.putNumber("ElevD", m_motorMaster.configAccessor.closedLoop.getD());
        SmartDashboard.putNumber("ElevV", m_feedForward.getKv());
   
    }

    private void logData(){
        SmartDashboard.putNumber("Elev Position", getPosition());
        SmartDashboard.putNumber("Elev Velocity", getVelocity());
        SmartDashboard.putNumber("Setpoint Pos", m_setpoint.position);
        SmartDashboard.putNumber("goal velocity", m_setpoint.velocity);
        SmartDashboard.putNumber("FF Voltage", m_feedForward.calculate(m_setpoint.velocity));
        
    }

    //ELEVATOR COMMANDS
    /*public Command manualElevCommand(double speed) {  //input values between -0.5 and 0.5
        return this.run(() -> {
            if (ElevatorConstants.kMaxHeight <= getPosition() && speed > 0) {
                m_motorMaster.set(0);
            } else if (getPosition() <= ElevatorConstants.kMinHeight && speed < 0) {
                m_motorMaster.set(0);
            } else {
                m_motorMaster.set(speed);
            }
        });
        }*/

    public Command getManualElevCommand(double speed) {
        return new FunctionalCommand(
            () -> {},
            () -> m_motorMaster.set(speed),
            interrupted -> m_motorMaster.set(0),
            () ->  ElevatorConstants.kMaxHeight <= getPosition(),
            this);
    }

    public Command getStopElevCommand() {
        return this.run(() -> m_motorMaster.set(0));
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    public void setVelocity(double velocity) {
        m_pidController.setReference(velocity, ControlType.kVelocity);
    }   

    //ELEVATOR MOTION PROFILE | for setpoints
    public void initializeSetpoint(){ //tells code what the actual position is    
        m_setpoint = new TrapezoidProfile.State(getPosition(), m_encoder.getVelocity());
    }
    
    public void setGoal(double desiredPosition, double desiredVelocity){
        m_goal = new TrapezoidProfile.State(desiredPosition, desiredVelocity);
    }
//print position of goal and elevator to tune kA
    private void goToSetpoint() {
        m_setpoint = m_profile.calculate(0.02, m_setpoint, m_goal);
        m_pidController.setReference(m_setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, m_feedForward.calculate(m_setpoint.velocity));
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(m_goal.position, getPosition(), 0.1);
    }

    public Command getSetpointCommand(double setpoint) {
        return new FunctionalCommand(
            () -> {
                initializeSetpoint();
                setGoal(setpoint, 0);
            
            },
            () -> {
                
                goToSetpoint();
            },
            interrupted -> applyAntiGravityFF(),
            () -> atSetpoint(),
            this);
    }

    public void applyAntiGravityFF(){
        m_motorMaster.setVoltage(m_feedForward.calculate(0));
    }

    public Command applyAntiGravityFFCommand(){
        return this.run(this::applyAntiGravityFF);
    }

    public Command getResetEncoderCommand() {
        return this.run(() -> resetEncoder());
    }

    private  void resetEncoder() {
        m_encoder.setPosition(0.0);
    }



    
   /*  private void tunePIDSmartDashboard() {
        double kP = SmartDashboard.getNumber("ElevP", 0.0);
        double kI = SmartDashboard.getNumber("ElevI", 0.0);
        double kD = SmartDashboard.getNumber("ElevD", 0.0);
        double kV = SmartDashboard.getNumber("ElevV", 0.0);
        
        if (kP != m_motorMaster.configAccessor.closedLoop.getP() ||
            kI != m_motorMaster.configAccessor.closedLoop.getI() ||
            kD != m_motorMaster.configAccessor.closedLoop.getD() ||
            kV != m_feedForward.getKv()) {
            ElevatorConfigs.kMasterConfig.closedLoop.pid(kP, kI, kD);
            ElevatorConfigs.kSlaveConfig.closedLoop.pid(kP, kI, kD);
            m_feedForward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, kV);
           
            ElevatorConstants.kV = kV;
            ElevatorConstants.kP = kP;
            ElevatorConstants.kI = kI;
            ElevatorConstants.kD = kD;
            

            m_motorMaster.configure(ElevatorConfigs.kMasterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_motorSlave.configure(ElevatorConfigs.kSlaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }*/



    @Override
    public void periodic() {
        logData();
        //tunePIDSmartDashboard();
    
    }

}