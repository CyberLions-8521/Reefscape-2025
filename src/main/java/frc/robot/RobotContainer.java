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

public class RobotContainer {
  private final Elevator m_elevator = new Elevator(ElevatorConstants.kMasterID, ElevatorConstants.kSlaveID);
  private final Shooter m_shooter = new Shooter(ShooterConstants.kMasterID, ShooterConstants.kSlaveID);
  private final CommandXboxController m_driveController = new CommandXboxController(OperaterConstants.kDriveControllerPort);
  private final CommandXboxController m_commandController = new CommandXboxController(OperaterConstants.kCommandControllerPort);
  private final Swerve m_db = new Swerve();

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
    //m_controller.a().onTrue(new ElevatorGoToSetpoint(.30, m_elevator));
    m_commandController.rightTrigger().whileTrue(new ElevatorUp(m_elevator, .6));
    m_commandController.leftTrigger().whileTrue(new ElevatorDown(m_elevator, -.6));

    m_commandController.rightBumper().whileTrue(new ElevatorUp(m_elevator, .23));
   m_commandController.leftBumper().whileTrue(new ElevatorDown(m_elevator, -.23));
    m_commandController.povLeft().onTrue(new ElevatorGoToL2(0, m_elevator, 0.9));
    m_commandController.povUp().onTrue(new ElevatorGoToL3(0, m_elevator, 0.9));
    m_commandController.povRight().onTrue(new ElevatorGoToL4(0, m_elevator, 0.9));

    //m_commandController.x().onTrue(new ElevatorGoToSetpoint(0, m_elevator, 0.9));
    //m_controller.a().onTrue(m_elevator.resetEncoderCommand());

    m_commandController.b().onTrue(new Intake(m_shooter, 14.5)); //intakes
    m_commandController.y().onTrue(new Intake(m_shooter, 2.21232)); //intake assist

    m_commandController.a().whileTrue(new Shoot(m_shooter, .5)); //shoots slow
    //m_commandController.x().whileTrue(new Shoot(m_shooter, 1.0)); //shoots faster
    m_shooter.register();


    

    // m_XboxController.a().onTrue(new InstantCommand(m_db::setSpeed1, m_db));
    // m_XboxController.x().onTrue(new InstantCommand(m_db::setSpeed2, m_db));
    // m_XboxController.y().onTrue(new InstantCommand(m_db::stopMotors, m_dx`b));
    m_driveController.b().onTrue(m_db.resetEncodersCommand());
    m_driveController.a().onTrue(m_db.resetGyroCommand());
    
    //tune command - right trigger
    // m_driveController.rightTrigger().whileTrue(getDriveCommand (
    //         1.0,
    //         m_driveController::getLeftY, 
    //         m_driveController::getLeftX, 
    //         m_driveController::getRightX, 
    //         m_driveController.getHID()::getRightBumperButton));

    //regular drive with slew rate applied
    m_db.setDefaultCommand(getDriveCommand (
            1,
            m_driveController::getLeftY, 
            m_driveController::getLeftX, 
            m_driveController::getRightX, 
            m_driveController.getHID()::getRightBumperButton));
    
    //brake driving - left trigger
    m_driveController.leftTrigger().whileTrue(getDriveCommand (
            0.5,
            m_driveController::getLeftY, 
            m_driveController::getLeftX, 
            m_driveController::getRightX, 
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



    
  public Command getDriveCommand(double multiplier, Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> omega, Supplier<Boolean> fieldRelative) {
    // double xSpeed = -MathUtil.applyDeadband(vx.get(), ControllerConstants.kDeadband);
    // xSpeed = vx_limiter.calculate(Math.copySign(xSpeed * xSpeed, xSpeed));
    // final double xSpeedDelivered = xSpeed * multiplier * SwerveDrivebaseConstants.kMaxMetersPerSecond;

    // double ySpeed = -MathUtil.applyDeadband(vy.get(), ControllerConstants.kDeadband);
    // ySpeed = vy_limiter.calculate(Math.copySign(ySpeed * ySpeed, ySpeed));
    // final double ySpeedDelivered = ySpeed * multiplier * SwerveDrivebaseConstants.kMaxMetersPerSecond;

    // double rOmega = -MathUtil.applyDeadband(omega.get(), ControllerConstants.kDeadband);
    // rOmega = omega_limiter.calculate(Math.copySign(rOmega * rOmega, rOmega));
    // final double omegaDelivered = (rOmega * multiplier) * SwerveDrivebaseConstants.kMaxAngularSpeed;
    
    // return new RunCommand(
    //   () -> m_db.drive(
    //     (xSpeedDelivered),
    //     (ySpeedDelivered),
    //     (omegaDelivered ),
    //     !fieldRelative.get()),
    //   m_db); //omega_limiter.calculate(
    return new RunCommand(
        () -> m_db.drive(
          -vx_limiter.calculate(Math.copySign(MathUtil.applyDeadband(vx.get(),ControllerConstants.kDeadband) * MathUtil.applyDeadband(vx.get(), ControllerConstants.kDeadband), vx.get()))
            * multiplier * SwerveDrivebaseConstants.kMaxMetersPerSecond,
          -vy_limiter.calculate(Math.copySign(MathUtil.applyDeadband(vy.get(),ControllerConstants.kDeadband) * MathUtil.applyDeadband(vy.get(), ControllerConstants.kDeadband), vy.get()))
            * multiplier * SwerveDrivebaseConstants.kMaxMetersPerSecond,
          -omega_limiter.calculate(Math.copySign(MathUtil.applyDeadband(omega.get(),ControllerConstants.kDeadband) * MathUtil.applyDeadband(omega.get(), ControllerConstants.kDeadband), omega.get()))
            * multiplier * SwerveDrivebaseConstants.kMaxMetersPerSecond,
          !fieldRelative.get()),
        m_db); //omega_limiter.calculate(
      
  }

  public Command getTuneDriveCommand(Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> omega, Supplier<Boolean> fieldRelative, double multiplier) {
    return new RunCommand(
      () -> m_db.drive(
        -MathUtil.applyDeadband(vx.get() , ControllerConstants.kDeadband) * multiplier * SwerveDrivebaseConstants.kMaxMetersPerSecond,
        -MathUtil.applyDeadband(vy.get() , ControllerConstants.kDeadband) * multiplier * SwerveDrivebaseConstants.kMaxMetersPerSecond,
        -MathUtil.applyDeadband(omega.get() , ControllerConstants.kDeadband) * multiplier * SwerveDrivebaseConstants.kMaxAngularSpeed,
        !fieldRelative.get()),
      m_db); //omega_limiter.calculate(
  }
  
}
