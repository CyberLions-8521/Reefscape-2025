 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.OperaterConstants;
import frc.robot.Constants.SwerveDrivebaseConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;

public class RobotContainer {
  private final Swerve m_db = new Swerve();
  private final Elevator m_elevator = new Elevator(ElevatorConstants.kMaster, ElevatorConstants.kSlave);
  private final Shooter m_shooter = new Shooter(ShooterConstants.kMasterID, ShooterConstants.kSlaveID);
  private final CommandXboxController m_driveController = new CommandXboxController(OperaterConstants.kDriveControllerPort);
  private final CommandXboxController m_commandController = new CommandXboxController(OperaterConstants.kCommandControllerPort);
  private final SlewRateLimiter vx_limiter = new SlewRateLimiter(SwerveDrivebaseConstants.kSlewRateLimiter);
  private final SlewRateLimiter vy_limiter = new SlewRateLimiter(SwerveDrivebaseConstants.kSlewRateLimiter);
  private final SlewRateLimiter omega_limiter = new SlewRateLimiter(SwerveDrivebaseConstants.kSlewRateLimiter);
  private final SendableChooser<Command> m_chooser = new SendableChooser<Command>();

  public RobotContainer() {
    configureBindings();
    SmartDashboard.putData(m_chooser);
  }

 
  private void configureBindings() {
 
    m_commandController.leftTrigger().whileTrue(m_shooter.getShootCommand(-0.2));
    m_commandController.rightTrigger().whileTrue(m_shooter.getShootCommand(0.2));

    m_commandController.povUp().onTrue(m_shooter.getIntakeCommand(13.0));

    m_commandController.leftBumper().whileTrue(m_elevator.getManualElevCommand(-0.1));
    m_commandController.rightBumper().whileTrue(m_elevator.getManualElevCommand(0.2));

    m_commandController.y().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kL4Setpoint));
    m_commandController.x().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kL3Setpoint));
    m_commandController.a().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kL2Setpoint));
    m_commandController.b().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kL1Setpoint));
    
    m_shooter.register();

    m_driveController.b().onTrue(m_db.resetEncodersCommand());
    m_driveController.a().onTrue(m_db.resetGyroCommand());

    m_elevator.setDefaultCommand(m_elevator.applyAntiGravityFFCommand());

    // regular drive with slew rate applied
    m_db.setDefaultCommand(getDriveCommand(
      1,
      getJoystickValues(m_driveController::getLeftY, vx_limiter),
      getJoystickValues(m_driveController::getLeftX, vy_limiter),
      getJoystickValues(m_driveController::getRightX, omega_limiter),
      m_driveController.getHID()::getRightBumperButton));
    
    // brake driving - left trigger
    m_driveController.leftTrigger().whileTrue(getDriveCommand(
      0.5,
      getJoystickValues(m_driveController::getLeftY, vx_limiter),
      getJoystickValues(m_driveController::getLeftX, vy_limiter),
      getJoystickValues(m_driveController::getRightX, omega_limiter),
      m_driveController.getHID()::getRightBumperButton));
    
   }

   private Command getDriveCommand(double multiplier, Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> omega, Supplier<Boolean> fieldRelative) {
    return new RunCommand(
      () -> m_db.drive(
        -vx.get() * multiplier * SwerveDrivebaseConstants.kMaxMetersPerSecond,
        -vy.get() * multiplier * SwerveDrivebaseConstants.kMaxMetersPerSecond,
        -omega.get() * multiplier * SwerveDrivebaseConstants.kMaxMetersPerSecond,
        !fieldRelative.get()),
      m_db);    
  }
  
  private Supplier<Double> getJoystickValues(Supplier<Double> controller, SlewRateLimiter limiter) {
    return () -> {
      double deadBandValue = MathUtil.applyDeadband(controller.get(), ControllerConstants.kDeadband);
      double squaredValue = Math.copySign(deadBandValue * deadBandValue, deadBandValue);
      return limiter.calculate(squaredValue);
    };
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}