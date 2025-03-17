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
import frc.robot.Commands.ElevatorGoToSetpoint;
import frc.robot.Commands.ElevatorGoToL2;
import frc.robot.Commands.ElevatorGoToL3;
import frc.robot.Commands.ElevatorGoToL4;
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
import frc.robot.Subsystems.Limelight;

public class RobotContainer {
  private final Elevator m_elevator = new Elevator(ElevatorConstants.kMaster, ElevatorConstants.kSlave);
  private final Shooter m_shooter = new Shooter(ShooterConstants.kMasterID, ShooterConstants.kSlaveID);
  private final CommandXboxController m_driveController = new CommandXboxController(OperaterConstants.kDriveControllerPort);
  private final CommandXboxController m_commandController = new CommandXboxController(OperaterConstants.kCommandControllerPort);
  private final Swerve m_db = new Swerve();
  private final Limelight limelight = new Limelight();

  SlewRateLimiter vx_limiter = new SlewRateLimiter(SwerveDrivebaseConstants.kSlewRateLimiter);
  SlewRateLimiter vy_limiter = new SlewRateLimiter(SwerveDrivebaseConstants.kSlewRateLimiter);
  SlewRateLimiter omega_limiter = new SlewRateLimiter(SwerveDrivebaseConstants.kSlewRateLimiter);
  
  

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    configureAutos();

    SmartDashboard.putData(m_chooser);
  }
 
  private void configureBindings() {
    m_commandController.rightTrigger().whileTrue(new ElevatorUp(m_elevator, .63));
    m_commandController.leftTrigger().whileTrue(new ElevatorDown(m_elevator, -.63));

    m_commandController.rightBumper().whileTrue(new ElevatorUp(m_elevator, .23));
    m_commandController.leftBumper().whileTrue(new ElevatorDown(m_elevator, -.23));


    m_commandController.b().onTrue(new Intake(m_shooter, 14.5)); //intakes
    m_commandController.y().onTrue(new Intake(m_shooter, 2.21232)); //intake assist

    m_commandController.a().whileTrue(new Shoot(m_shooter, .45)); //shoots slow
    m_commandController.x().whileTrue(new Shoot(m_shooter, 1.0)); //shoots faster
    m_shooter.register();
    // limelight.register();

    m_driveController.b().onTrue(m_db.resetEncodersCommand());
    m_driveController.a().onTrue(m_db.resetGyroCommand());

    //regular drive with slew rate applied
    m_db.setDefaultCommand(getDriveCommand (
            1,
            getJoystickValues(m_driveController::getLeftY, vx_limiter),
            getJoystickValues(m_driveController::getLeftX, vy_limiter),
            getJoystickValues(m_driveController::getRightX, omega_limiter),
            m_driveController.getHID()::getRightBumperButton));
    
    //brake driving - left trigger
    m_driveController.leftTrigger().whileTrue(getDriveCommand (
            0.5,
            getJoystickValues(m_driveController::getLeftY, vx_limiter),
            getJoystickValues(m_driveController::getLeftX, vy_limiter),
            getJoystickValues(m_driveController::getRightX, omega_limiter),
            m_driveController.getHID()::getRightBumperButton));
   }

  public void configureAutos() {
    m_chooser.setDefaultOption("No Auto", null);
    m_db.resetGyro();
    m_chooser.addOption("Drive Straight", new DriveToDistance(m_db, 4));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public Supplier<Double> getJoystickValues(Supplier<Double> controller, SlewRateLimiter limiter) {
    return () -> {
      double deadBandValue = MathUtil.applyDeadband(controller.get(), ControllerConstants.kDeadband);
      double squaredValue = Math.copySign(deadBandValue * deadBandValue, deadBandValue);
      return limiter.calculate(squaredValue);
    };
  }

    
  public Command getDriveCommand(double multiplier, Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> omega, Supplier<Boolean> fieldRelative) {
    return new RunCommand(
        () -> m_db.drive(
          -vx.get() * multiplier * SwerveDrivebaseConstants.kMaxMetersPerSecond,
          -vy.get() * multiplier * SwerveDrivebaseConstants.kMaxMetersPerSecond,
          -omega.get() * multiplier * SwerveDrivebaseConstants.kMaxMetersPerSecond,
          !fieldRelative.get()),
        m_db); 
      
  }
  
}
