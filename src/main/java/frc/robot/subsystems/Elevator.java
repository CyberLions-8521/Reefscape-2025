package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.MotorConfigs;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);
    SparkClosedLoopController closedLoopController;

    //private TalonFX m_motor;
    private SparkMax m_motorMaster;
    private SparkMax m_motorSlave;

    private RelativeEncoder m_encoder;

    ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kS,
          ElevatorConstants.kG,
          ElevatorConstants.kV,
          ElevatorConstants.kA);

    public Elevator(int masterMotorPort, int slaveMotorPort) {
        m_motorMaster = new SparkMax(masterMotorPort, MotorType.kBrushless);
        m_motorSlave = new SparkMax(slaveMotorPort, MotorType.kBrushless);

        m_motorMaster.configure(MotorConfigs.ELEV_MASTER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_motorSlave.configure(MotorConfigs.ELEV_SLAVE_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder = m_motorMaster.getEncoder();

        closedLoopController = m_motorMaster.getClosedLoopController();

        SmartDashboard.putNumber("kG", 0);
    }

    public double getPosition() { //INCHES
        return m_encoder.getPosition();
    }

    public void goToSetpoint(double setpoint){
        closedLoopController.setReference(setpoint,
         ControlType.kPosition, ClosedLoopSlot.kSlot0, m_feedforward.calculate(0));
    }

    public Command getSetpointCommand() {
      return this.run(() -> goToSetpoint(getPosition()));
    }

    public SparkClosedLoopController getController() {
        return closedLoopController;
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
        m_encoder.setPosition(0.00);
    }

    public void logData() {
        SmartDashboard.putNumber("Elevator Position", m_encoder.getPosition());
    }

    @Override
    public void periodic() {
        logData();
        // tunekG();
    }

    public void tunekG() {
        double kG = SmartDashboard.getNumber("kG", 0);

        if(ElevatorConstants.kG != kG) {
            // ElevatorConstants.kG = kG;
            m_feedforward = new ElevatorFeedforward(0, kG, 0);
        }


    }

}