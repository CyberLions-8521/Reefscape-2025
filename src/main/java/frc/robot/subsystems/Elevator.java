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
  private final SparkMax                  m_master        = new SparkMax(Constants.ElevatorConstants.kMotor1, MotorType.kBrushless);
  private final SparkMax                  m_slave        = new SparkMax(Constants.ElevatorConstants.kMotor2, MotorType.kBrushless);
  private final SparkClosedLoopController m_controller    = m_motor1.getClosedLoopController();
  private final RelativeEncoder           m_encoder       = m_motor1.getEncoder();
  private final RelativeEncoder           m_encoder2      = m_motor2.getEncoder();
  private final SparkMaxConfig            m_masterConfig = new SparkMaxConfig();
  private final SparkMaxConfig            m_slaveConfig = new SparkMaxConfig();
  double m_desiredHeight;

  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kElevatorkS,
          ElevatorConstants.kElevatorkG,
          ElevatorConstants.kElevatorkV,
          ElevatorConstants.kElevatorkA);
  
  // Constructor
  public Elevator()
  {
     //Set up motor configs
    m_masterConfig.encoder
            .positionConversionFactor(ElevatorConstants.kRotationToMeters) // Converts Rotations to Meters
            .velocityConversionFactor(ElevatorConstants.kRotationToMeters / 60); // Converts RPM to MPS
    m_masterConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd, ClosedLoopSlot.kSlot0)//Change PID with these constants.
            .pid(ElevatorConstants.kSlowElevatorKp, ElevatorConstants.kSlowElevatorKi, ElevatorConstants.kSlowElevatorKd, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
            .outputRange(-0.3, 0.3, ClosedLoopSlot.kSlot1);
    m_masterConfig.closedLoop.maxMotion
            .maxVelocity(ElevatorConstants.kElevatorMaxVelocity)
            .maxAcceleration(ElevatorConstants.kElevatorMaxAcceleration)
            .allowedClosedLoopError(Units.inchesToMeters(0.1)); // TODO: Tune this as we go -see what's reasonable

    m_masterConfig.idleMode(IdleMode.kBrake);
    m_slaveConfig.idleMode(IdleMode.kBrake);

    m_masterConfig.smartCurrentLimit(Constants.ElevatorConstants.kMaxCurrent);
    m_slaveConfig.smartCurrentLimit(Constants.ElevatorConstants.kMaxCurrent);

    m_masterConfig.closedLoopRampRate(Constants.ElevatorConstants.kRampRate);
    m_slaveConfig.closedLoopRampRate(Constants.ElevatorConstants.kRampRate);


 //Configure motors
    m_masterConfig.disableFollowerMode();
    m_master.configure(m_masterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_slaveConfig.follow(m_master,true);
    m_slave.configure(m_slaveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

 // desired height
    m_desiredHeight = 0;

  }

  public void motorStop(){
    m_motor1.set(0);
  }

  public void resetEncoder(){
    m_encoder.setPosition(0);
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal)
  {
    m_desiredHeight = goal;
    if (m_desiredHeight == Constants.ElevatorConstants.kL4PostScoringHeightMeters) {
      m_controller.setReference(goal,
                              ControlType.kPosition,
                              ClosedLoopSlot.kSlot1,
                              m_feedforward.calculate(m_encoder.getVelocity()));
    }

    else {
      m_controller.setReference(goal,
                              ControlType.kPosition,
                              ClosedLoopSlot.kSlot0,
                              m_feedforward.calculate(m_encoder.getVelocity()));
    }

  }

  /**
   * Get the height in meters.
   *
   * @return Height in meters
   */
  public double getHeight()
  {
    return m_encoder.getPosition();
  }

  /**
   * A trigger for when the height is at an acceptable tolerance.
   *
   * @param height    Height in Meters
   * @param tolerance Tolerance in meters.
   * @return {@link Trigger}
   */
  public Trigger atHeight(double height, double tolerance)
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
  public Command setGoal(double goal)
  {
    return run(() -> reachGoal(goal));
  }

  /**
   * Stop the control loop and motor output.
   */
  public void stop()
  {
    m_motor1.set(0.0);
  }

  //These are good to use the set function
  public void ManualElevatorUp(){
    m_motor1.set(0.2);
  }

  public void HoverElevator(){
    m_motor1.setVoltage(ElevatorConstants.kElevatorkG);
  }
  
  public void ManualElevatorDown(){
    m_motor1.set(-0.15);
  }

  @Override
  public void periodic() {

    // Add useful info to dashboard(s)
    encoder1_publisher.set(m_encoder.getPosition());
    encoder2_publisher.set(m_encoder2.getPosition());
    output1_publisher.set(m_motor1.getAppliedOutput());
    output2_publisher.set(m_motor2.getAppliedOutput());
    heightError_publisher.set(m_desiredHeight - getHeight());
    velocity_publisher.set(m_encoder.getVelocity());
    current1_publisher.set(m_motor1.getOutputCurrent());
    current2_publisher.set(m_motor2.getOutputCurrent());

    //updateTelemetry();
    if (m_encoder.getVelocity() < 0.01 && m_motor1.getOutputCurrent() > Constants.ElevatorConstants.kResetCurrent) {
      if (m_motor1.getAppliedOutput() > 0) {
        m_encoder.setPosition(Constants.ElevatorConstants.kMaxRealElevatorHeightMeters);
      }
      else {
        m_encoder.setPosition(Constants.ElevatorConstants.kMinRealElevatorHeightMeters);
      }
      m_motor1.set(0);
    }  
    SmartDashboard.putNumber("Elevator Position (Meters)", m_encoder.getPosition());
  }

}