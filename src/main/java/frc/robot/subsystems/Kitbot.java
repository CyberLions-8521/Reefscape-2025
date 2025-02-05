// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KitbotConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Kitbot extends SubsystemBase {
  /** Creates a new Kitbot. */
  public Kitbot() {
    SparkMax rollerMotor = new SparkMax(KitbotConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.voltageCompensation(KitbotConstants.ROLLER_MOTOR_VOLTAGE_COMP);
    rollerConfig.smartCurrentLimit(KitbotConstants.ROLLER_MOTOR_CURRENT_LIMIT);
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runRoller(){
    //rollerMotor.set
  }
}
