// Copyright (c) FIRST and other WPILib contributors.
   // Open Source Software; you can modify and/or share it under the terms of
   // the WPILib BSD license file in the root directory of this project.
 
   package frc.robot.Subsystems;
 
   import com.revrobotics.RelativeEncoder;
 import com.revrobotics.spark.SparkBase.PersistMode;
 import com.revrobotics.spark.SparkBase.ResetMode;
 import com.revrobotics.spark.SparkLowLevel.MotorType;
 import com.revrobotics.spark.SparkMax;
 
 import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.FunctionalCommand;
 import edu.wpi.first.wpilibj2.command.StartEndCommand;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import frc.robot.Configs.AlgaeConfig;
 
   public class Algae extends SubsystemBase {
     /** Creates a new Algae. */
     private SparkMax m_motor;
     private RelativeEncoder m_encoder;
 
     private double defaultState = 0.095;
     private double bellypanState = 0.261905;
 
     public Algae(int motorID) {
       m_motor = new SparkMax(motorID, MotorType.kBrushless);
       m_encoder = m_motor.getEncoder();
       m_motor.configure(AlgaeConfig.kAlgaeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
 
       resetEncoder();
     }
 
     //command to literatlly just run the motor to take off the algae
     public Command algaeUp() {
           return new FunctionalCommand(
               () -> {
                 this.m_encoder.setPosition(0.0);
                 this.m_motor.set(.34);
               },
               () -> {},
               interrupted -> this.m_motor.set(0.0),
               () -> atLimit(),
               this);
           }
 
     public Command algaeDown() {
           return new FunctionalCommand(
             () -> {
               this.m_motor.set(-.34);
             },
             () -> {},
             interrupted -> this.m_motor.set(0.0),
             () -> atLimit(),
             this);
       }
 
     public Command move(double speed) {
       return this.run(() -> m_motor.set(speed));   
     }
 
     public Command upForever(double speed) {
       return new StartEndCommand(
         () -> m_motor.set(speed),
         () -> m_motor.set(0.0),
         this);
 }
 
     public Command zeroEncoder() {
       return this.runOnce(() -> m_encoder.setPosition(0.0));
     }
 
   private boolean atLimit() {
             //runs when NOT greater or equal to than the negative limit or less than or equal to the max limit
           return (!(((m_motor.get() < 0.0) && (m_motor.get() >= bellypanState)) || 
             (m_motor.get() > 0.0) && (m_motor.get() <= defaultState)));
   }
 
   private  void resetEncoder() {
         m_encoder.setPosition(0.0);
     }
 
     @Override
     public void periodic() {
       // This method will be called once per scheduler run
       logData();
     }
 
     private void logData() {
       SmartDashboard.putNumber("algae position", m_encoder.getPosition());
     }
   }