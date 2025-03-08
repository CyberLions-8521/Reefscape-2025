package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase
{
  // Set up elevator properties
  private final SparkMax m_master = new SparkMax(Constants.ElevatorConstants.kMaster, MotorType.kBrushless);
  private final SparkMax m_slave = new SparkMax(Constants.ElevatorConstants.kSlave, MotorType.kBrushless);
  private final SparkClosedLoopController m_controller = m_master.getClosedLoopController();
  private final RelativeEncoder m_encoderMaster = m_master.getEncoder();
  private final RelativeEncoder m_encoderSlave = m_slave.getEncoder();

  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kElevatorkS,
          ElevatorConstants.kElevatorkG,
          ElevatorConstants.kElevatorkV,
          ElevatorConstants.kElevatorkA);
  
  // Constructor
  public Elevator()
  {

    //Configure motors
    m_masterConfig.disableFollowerMode();
    m_master.configure(ElevatorConstants.m_masterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_slaveConfig.follow(m_master,true);
    m_slave.configure(ElevatorConstants.m_slaveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void stopMotors(){
    m_master.set(0);
  }

  public void resetEncoder(){
    m_encoder.setPosition(0);
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void goToSetpoint(double height)
  {
      m_controller.setReference(height,
        ControlType.kPosition, 
        0,
        m_feedforward.calculate(0)));
        //m_encoder.getVelocity()));
  }

  public double getPosition()
  {
    return m_encoder.getPosition();
  }

 
  public Trigger atPosition(double height, double tolerance)
  {
    return new Trigger(() -> MathUtil.isNear(height,
                                             getHeight(),
                                             tolerance));
  }

  /**
   * Set the goal of the elevator
   *
   * @param goal Goal in meters
   * @return {@link edu.wpi.first.wpilibj2.command.Command}
   */
  public Command setSetpoint(double position)
  {
    return run(() -> goToSetpoint(position));
  }

  /**
   * Stop the control loop and motor output.
   */
  

  //These are good to use the set function
  public void ElevatorUp(){
    m_master.set(0.2);
  }

  public void HoverElevator(){
    m_master.setVoltage(ElevatorConstants.kElevatorkG);
  }
  
  public void ElevatorDown(){
    m_master.set(-0.15);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("master position", m_encoderMaster.getPosition());
    SmartDashboard.putNumber("slave position", m_encoderSlave.getPosition())

  }

}