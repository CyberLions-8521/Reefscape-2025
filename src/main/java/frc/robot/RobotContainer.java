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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperaterConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveDrivebaseConstants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subs.Elevator;
import frc.robot.subs.Shooter;
import frc.robot.commands.AutoAlignToReefLeft;
import frc.robot.commands.AutoAlignToReefRight;
import frc.robot.commands.LiftAlgae;
import frc.robot.subs.Algae;
import frc.robot.subs.Climber;
import frc.robot.subs.LimelightTester;
import frc.robot.subs.Swerve;

public class RobotContainer {
  private final Swerve m_db = new Swerve();
  private final Elevator m_elevator = new Elevator(ElevatorConstants.kMaster, ElevatorConstants.kSlave);
  private final Shooter m_shooter = new Shooter(ShooterConstants.kMasterID, ShooterConstants.kSlaveID);
  private final Climber m_climber = new Climber(ClimberConstants.kMotorID);
  private final CommandXboxController m_driveController = new CommandXboxController(OperaterConstants.kDriveControllerPort);
  private final CommandXboxController m_commandController = new CommandXboxController(OperaterConstants.kCommandControllerPort);
  private final SlewRateLimiter vx_limiter = new SlewRateLimiter(SwerveDrivebaseConstants.kSlewRateLimiter);
  private final SlewRateLimiter vy_limiter = new SlewRateLimiter(SwerveDrivebaseConstants.kSlewRateLimiter);
  private final SlewRateLimiter omega_limiter = new SlewRateLimiter(SwerveDrivebaseConstants.kSlewRateLimiter);
  private final Algae m_algae = new Algae(AlgaeConstants.kMotorID);
  private final SendableChooser<Command> m_chooser = new SendableChooser<Command>();
  // private final LimelightTester limelight = new LimelightTester(0);

  public RobotContainer() {
    configureBindings();
    configureAutos();
    SmartDashboard.putData(m_chooser);
  }

 
  private void configureBindings() {
    //SUBSYSTEMS CONTROLLER
    m_commandController.leftTrigger().whileTrue(m_shooter.getShootCommand(-0.3));
    m_commandController.rightTrigger().whileTrue(m_shooter.getShootCommand(0.45));
    
    m_commandController.povUp().onTrue(m_shooter.getIntakeCommand(4.7));
    m_commandController.povRight().whileTrue(m_algae.move(0.5));
    m_commandController.povLeft().whileTrue(m_algae.move(-0.2));
    m_commandController.start().onTrue(m_algae.upForever(0.05));
    m_commandController.povDown().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint));

    m_commandController.leftBumper().whileTrue(m_elevator.getManualElevCommand(-0.1));
    m_commandController.rightBumper().whileTrue(m_elevator.getManualElevCommand(0.2));
    
    m_commandController.y().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kL4Setpoint));
    m_commandController.x().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kL3Setpoint));
    m_commandController.a().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kL2Setpoint));
    m_commandController.b().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kL1Setpoint));

    
    m_shooter.register();
    // limelight.register();

    //SWERVE CONTROLLER
    m_driveController.b().onTrue(m_db.resetEncodersCommand());
    m_driveController.a().onTrue(m_db.resetGyroCommand());
    
    m_driveController.povLeft().onTrue(new AutoAlignToReefLeft(m_db, LimelightConstants.kDistanceToReefLeft));
    m_driveController.povRight().onTrue(new AutoAlignToReefRight(m_db, -LimelightConstants.kDistanceToReefRight));

    m_driveController.povUp().whileTrue(m_climber.moveClimberCommand(0.5));
    m_driveController.povDown().whileTrue(m_climber.moveClimberCommand(-0.5));

    m_climber.setDefaultCommand(m_climber.moveClimberCommand(0));



    m_elevator.setDefaultCommand(m_elevator.applyAntiGravityFFCommand());

    // regular drive with slew rate applied
    m_db.setDefaultCommand(this.getDriveCommand(
      1,
      getJoystickValues(m_driveController::getLeftY, vx_limiter),
      getJoystickValues(m_driveController::getLeftX, vy_limiter),
      getJoystickValues(m_driveController::getRightX, omega_limiter),
      m_driveController.getHID()::getRightBumperButton));
    
    // brake driving - left trigger
    m_driveController.leftTrigger().whileTrue(this.getDriveCommand(
      0.5,
      getJoystickValues(m_driveController::getLeftY, vx_limiter),
      getJoystickValues(m_driveController::getLeftX, vy_limiter),
      getJoystickValues(m_driveController::getRightX, omega_limiter),
      m_driveController.getHID()::getRightBumperButton));

    m_elevator.setDefaultCommand(m_elevator.applyAntiGravityFFCommand());
    
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

  private void configureAutos() {
    m_chooser.setDefaultOption("No Auto", null);
    //m_chooser.addOption("Algae Up", new LiftAlgae(m_algae, m_elevator));
    m_chooser.addOption("Taxi", new SequentialCommandGroup(
      m_db.resetEncodersCommand(),
      m_db.getDriveCommand(4), //estimated sum of distance between starting line and driver side of reef according to FIRST. robot-centric
      m_db.setGyroCommand(180)
    ));

    m_chooser.addOption("Pos2L2Auto", new SequentialCommandGroup(
      //moves to reef while elevator goes to L2
      m_db.resetEncodersCommand(),
      new ParallelCommandGroup( 
        m_db.getDriveCommand(2), //estimated distance between reef and starting line according to FIRST (https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.pdf) 
        m_elevator.getSetpointCommand(ElevatorConstants.kL2Setpoint)
      ),
      //align and shoot
      new AutoAlignToReefLeft(m_db, 0),
      m_shooter.getShootCommand(0.5),
      //reverse gyro for teleop
      m_db.setGyroCommand(180)
    ));

    m_chooser.addOption("Pos2L4Auto", new SequentialCommandGroup(
      //moves to reef while elevator goes to L4
      m_db.resetEncodersCommand(),
      new ParallelCommandGroup(
        m_db.getDriveCommand(2),
        m_elevator.getSetpointCommand(ElevatorConstants.kL4Setpoint)
      ),
      new AutoAlignToReefLeft(m_db, 0),
      m_shooter.getShootCommand(0.5),
      m_db.setGyroCommand(180)
    ));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}