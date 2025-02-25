 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.fasterxml.jackson.databind.ser.std.StdArraySerializers.IntArraySerializer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveToDistance;
import frc.robot.Commands.ElevatorDown;
import frc.robot.Commands.ElevatorUp;
import frc.robot.Commands.Intake;
import frc.robot.Commands.Shoot;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.OperaterConstants;
import frc.robot.Constants.SwerveDrivebaseConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;

public class RobotContainer {
  private final Elevator m_elevator = new Elevator(ElevatorConstants.kMasterID, ElevatorConstants.kSlaveID);
  private final Shooter m_shooter = new Shooter(ShooterConstants.kMasterID, ShooterConstants.kSlaveID);
  private final CommandXboxController m_driveController = new CommandXboxController(OperaterConstants.kDriveControllerPort);
  private final CommandXboxController m_commandController = new CommandXboxController(OperaterConstants.kCommandControllerPort);
  private final Swerve m_db = new Swerve();

  SlewRateLimiter vx_limiter = new SlewRateLimiter(0.9);
  SlewRateLimiter vy_limiter = new SlewRateLimiter(0.9);
  //SlewRateLimiter omega_limiter = new SlewRateLimiter(1.0);
  
  

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    configureAutos();

    SmartDashboard.putData(m_chooser);
  }
 
  private void configureBindings() {
    //m_controller.a().onTrue(new ElevatorGoToSetpoint(.30, m_elevator));
    m_commandController.rightTrigger().whileTrue(new ElevatorUp(m_elevator, .5));
    m_commandController.leftTrigger().whileTrue(new ElevatorDown(m_elevator, -.5));
    //m_controller.a().onTrue(m_elevator.resetEncoderCommand());

    m_commandController.b().onTrue(new Intake(m_shooter, 14.5)); //intakes
    m_commandController.y().onTrue(new Intake(m_shooter, 2.21232)); //intake assist

    m_commandController.a().whileTrue(new Shoot(m_shooter, .4)); //shoots slow
    m_commandController.x().whileTrue(new Shoot(m_shooter, 1.0)); //shoots faster
    m_shooter.register();


    

    // m_XboxController.a().onTrue(new InstantCommand(m_db::setSpeed1, m_db));
    // m_XboxController.x().onTrue(new InstantCommand(m_db::setSpeed2, m_db));
    // m_XboxController.y().onTrue(new InstantCommand(m_db::stopMotors, m_dx`b));
    m_driveController.b().onTrue(m_db.resetEncodersCommand());
    m_driveController.a().onTrue(m_db.resetGyroCommand());
    
    //tune command - right trigger
    m_driveController.rightTrigger().whileTrue(getTuneDriveCommand (
            m_driveController::getLeftY, 
            m_driveController::getLeftX, 
            m_driveController::getRightX, 
            m_driveController.getHID()::getRightBumperButton,0.5));

    //regular drive with slew rate applied
    m_db.setDefaultCommand(getDriveCommand (
            m_driveController::getLeftY, 
            m_driveController::getLeftX, 
            m_driveController::getRightX, 
            m_driveController.getHID()::getRightBumperButton,1));
    
    //brake driving - left trigger
    m_driveController.leftTrigger().whileTrue(getDriveCommand (
            m_driveController::getLeftY, 
            m_driveController::getLeftX, 
            m_driveController::getRightX, 
            m_driveController.getHID()::getRightBumperButton,0.5));
   }

  public void configureAutos() {
    m_chooser.setDefaultOption("No Auto", null);
    m_chooser.addOption("Drive Straight", new DriveToDistance(m_db, 3));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }



    
  public Command getDriveCommand(Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> omega, Supplier<Boolean> fieldRelative, double multiplier) {
    return new RunCommand(
      () -> m_db.drive(
        -MathUtil.applyDeadband(vx_limiter.calculate(vx.get()) * multiplier, ControllerConstants.kDeadband) * SwerveDrivebaseConstants.kMaxMetersPerSecond,
        -MathUtil.applyDeadband(vy_limiter.calculate(vy.get()) * multiplier, ControllerConstants.kDeadband) * SwerveDrivebaseConstants.kMaxMetersPerSecond,
        -MathUtil.applyDeadband(omega.get() * multiplier, ControllerConstants.kDeadband) * SwerveDrivebaseConstants.kMaxAngularSpeed,
        !fieldRelative.get()),
      m_db); //omega_limiter.calculate(
  }

  public Command getTuneDriveCommand(Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> omega, Supplier<Boolean> fieldRelative, double multiplier) {
    return new RunCommand(
      () -> m_db.drive(
        -MathUtil.applyDeadband(vx.get() * multiplier, ControllerConstants.kDeadband) * SwerveDrivebaseConstants.kMaxMetersPerSecond,
        -MathUtil.applyDeadband(vy.get() * multiplier, ControllerConstants.kDeadband) * SwerveDrivebaseConstants.kMaxMetersPerSecond,
        -MathUtil.applyDeadband(omega.get() * multiplier, ControllerConstants.kDeadband) * SwerveDrivebaseConstants.kMaxAngularSpeed,
        !fieldRelative.get()),
      m_db); //omega_limiter.calculate(
  }
  
}
