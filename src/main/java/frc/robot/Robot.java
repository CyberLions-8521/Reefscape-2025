// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import org.ejml.dense.block.MatrixOps_DDRB;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

//a Timed Robot is a robot that 

public class Robot extends TimedRobot {
  private SparkMax m_leftMotor;
  private SparkMax m_rightMotor;
  private XboxController m_controller;

  private RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
  private RelativeEncoder m_rightEncoder = m_leftMotor.getEncoder();

  private SparkClosedLoopController m_leftPID = m_drive;
  private SparkClosedLoopController m_rightPID;

  public Robot() {
    m_leftMotor = new SparkMax(14, MotorType.kBrushless);
    m_rightMotor = new SparkMax(3, MotorType.kBrushless);

    m_controller = new XboxController(0);

    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(true);
    m_rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void disabledInit() {} //the setup for when we enter disabled mode -- no code runs in disabled

  @Override
  public void disabledPeriodic() {} //runs while disabled -- ignore

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {} //as soon as you hit enable in automous, it runs this code

  @Override
  public void autonomousPeriodic() {} //as long as you are in autonomous mode

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {} //runs when you turn on teleop

  @Override
  public void teleopPeriodic() {
    double left_stick = -m_controller.getLeftY();
    double right_stick = -m_controller.getRightY();
    
    tankDrive(left_stick, right_stick);

  } //runs as long as you are in 

  public void tankDrive(double left, double right) {
    m_leftMotor.set(left);
    m_rightMotor.set(right);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
