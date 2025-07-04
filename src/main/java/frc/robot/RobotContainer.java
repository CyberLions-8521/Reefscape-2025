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
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperaterConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveDrivebaseConstants;
import frc.robot.commands.AutoAlignToReefLeft;
import frc.robot.commands.AutoAlignToReefRight;
import frc.robot.subs.Algae;
import frc.robot.subs.Climber;
import frc.robot.subs.Elevator;
import frc.robot.subs.Shooter;
import frc.robot.subs.Swerve;

public class RobotContainer {
  private final Swerve m_db = new Swerve();
  private final Elevator m_elevator = new Elevator(ElevatorConstants.kMaster, ElevatorConstants.kSlave);
  private final Shooter m_shooter = new Shooter(ShooterConstants.kMasterID, ShooterConstants.kSlaveID);
  private final Climber m_climber = new Climber(ClimberConstants.kMotorID);
  private final CommandXboxController m_driveController = new CommandXboxController(OperaterConstants.kDriveControllerPort);
  //private final CommandXboxController m_commandController = new CommandXboxController(OperaterConstants.kCommandControllerPort);
  private final CommandPS4Controller m_commandController = new CommandPS4Controller(OperaterConstants.kCommandControllerPort);
  private final SlewRateLimiter vx_limiter = new SlewRateLimiter(SwerveDrivebaseConstants.kSlewRateLimiter);
  private final SlewRateLimiter vy_limiter = new SlewRateLimiter(SwerveDrivebaseConstants.kSlewRateLimiter);
  private final SlewRateLimiter omega_limiter = new SlewRateLimiter(SwerveDrivebaseConstants.kSlewRateLimiter);
  //private final Algae m_algae = new Algae(AlgaeConstants.kMotorID);
  private final SendableChooser<Command> m_chooser = new SendableChooser<Command>();
  // private final LimelightTester limelight = new LimelightTester(0);

  public RobotContainer() {
    configureBindings();
    configureAutos();
    SmartDashboard.putData(m_chooser);
  }

 
  private void configureBindings() {
    //SUBSYSTEMS CONTROLLER (XBOX)
    /*
    m_commandController.leftTrigger().whileTrue(m_shooter.getShootCommand(0.17));
    m_commandController.rightTrigger().whileTrue(m_shooter.getShootCommand(0.28));

    m_commandController.back().whileTrue(m_algae.move(0.5));
    m_commandController.start().whileTrue(m_algae.move(-0.3));

    m_commandController.povRight().whileTrue(m_shooter.getShootCommand(0.2));
    m_commandController.povLeft().whileTrue(m_shooter.getShootCommand(-0.2));

    m_commandController.start().onTrue(m_algae.upForever(0.05));

    m_commandController.leftBumper().whileTrue(m_elevator.getManualElevCommand(-0.1));
    m_commandController.rightBumper().whileTrue(m_elevator.getManualElevCommand(0.2));

    
    m_commandController.y().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kL4Setpoint));
    m_commandController.x().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kL3Setpoint));
    m_commandController.a().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kL2Setpoint));
    m_commandController.b().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kL1Setpoint));
    m_commandController.povDown().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint));
     */

    //SUBSYSTEMS CONTROLLER (PS4)
    m_commandController.L2().whileTrue(m_shooter.getShootCommand(0.17));
    m_commandController.R2().whileTrue(m_shooter.getShootCommand(0.28));

    //m_commandController.povUp().whileTrue(m_algae.move(0.5));
    m_commandController.povDown().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint));

    m_commandController.povRight().whileTrue(m_shooter.getShootCommand(0.2));
    m_commandController.povLeft().whileTrue(m_shooter.getShootCommand(-0.2));

    //m_commandController.options().onTrue(m_algae.upForever(0.05));

    m_commandController.L1().whileTrue(m_elevator.getManualElevCommand(-0.1));
    m_commandController.R1().whileTrue(m_elevator.getManualElevCommand(0.2));

    
    m_commandController.triangle().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kL4Setpoint));
    m_commandController.square().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kL3Setpoint));
    m_commandController.cross().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kL2Setpoint));
    m_commandController.circle().onTrue(m_elevator.getSetpointCommand(ElevatorConstants.kL1Setpoint));

    m_shooter.register();
    //m_limelight.register();

    //SWERVE CONTROLLER (XBOX)
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
    m_chooser.addOption("No Auto", null);
    //m_chooser.addOption("Algae Up", new LiftAlgae(m_algae, m_elevator));
    
    m_chooser.setDefaultOption("Forward Taxi", new SequentialCommandGroup(
      m_db.resetEncodersCommand(),
      m_db.resetGyroCommand(),
      m_db.getDriveCommand(1), //estimated sum of distance between starting line and driver side of reef according to FIRST. robot-centric
      m_db.setGyroCommand(180)
      ));

    m_chooser.addOption("Reversed Taxi", new SequentialCommandGroup(
      m_db.resetEncodersCommand(),
      m_db.resetGyroCommand(),
      m_db.getReversedDriveCommand(4) //estimated sum of distance between starting line and driver side of reef according to FIRST. robot-centric
    ));

    //left from driver station perspective
    m_chooser.addOption("L3.alignToLeft-left", new SequentialCommandGroup(
      m_db.resetEncodersCommand(),   
      m_db.resetGyroCommand(), 
      m_db.getDriveCommand(3).withTimeout(5),
      m_elevator.getSetpointCommand(ElevatorConstants.kL2Setpoint),
      new AutoAlignToReefLeft(m_db, LimelightConstants.kDistanceToReefLeft), 
      m_elevator.getSetpointCommand(ElevatorConstants.kL3Setpoint),          
      m_shooter.getShootCommand(0.17).withTimeout(2),
      m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint),
      m_db.resetEncodersCommand(),
      m_db.setGyroCommand(120)
    ));

    m_chooser.addOption("L3.alignToLeft-center", new SequentialCommandGroup(
      m_db.resetEncodersCommand(),   
      m_db.resetGyroCommand(), 
      m_db.getDriveCommand(3).withTimeout(5),
      m_elevator.getSetpointCommand(ElevatorConstants.kL2Setpoint),
      new AutoAlignToReefLeft(m_db, LimelightConstants.kDistanceToReefLeft), 
      m_elevator.getSetpointCommand(ElevatorConstants.kL3Setpoint),          
      m_shooter.getShootCommand(0.17).withTimeout(2),
      m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint),
      m_db.resetEncodersCommand(),
      m_db.setGyroCommand(180)
    ));

    //right from driver station perspective
    m_chooser.addOption("L3.alignToLeft-right", new SequentialCommandGroup(
      m_db.resetEncodersCommand(),   
      m_db.resetGyroCommand(), 
      m_db.getDriveCommand(3).withTimeout(5),
      m_elevator.getSetpointCommand(ElevatorConstants.kL2Setpoint),
      new AutoAlignToReefLeft(m_db, LimelightConstants.kDistanceToReefLeft), 
      m_elevator.getSetpointCommand(ElevatorConstants.kL3Setpoint),          
      m_shooter.getShootCommand(0.17).withTimeout(2),
      m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint),
      m_db.resetEncodersCommand(),
      m_db.setGyroCommand(240)
    ));

    m_chooser.addOption("L4.alignToLeft-left", new SequentialCommandGroup(
      m_db.resetEncodersCommand(),  
      m_db.resetGyroCommand(),   
      m_db.getDriveCommand(3).withTimeout(5),
      m_elevator.getSetpointCommand(ElevatorConstants.kL2Setpoint),
      new AutoAlignToReefLeft(m_db, LimelightConstants.kDistanceToReefLeft),  
      m_elevator.getSetpointCommand(ElevatorConstants.kL4Setpoint),          
      m_shooter.getShootCommand(0.28).withTimeout(2),
      m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint),
      m_db.resetEncodersCommand(),
      m_db.setGyroCommand(120)
    ));

    m_chooser.addOption("L4.alignToLeft-center", new SequentialCommandGroup(
      m_db.resetEncodersCommand(),  
      m_db.resetGyroCommand(),
      m_db.getDriveCommand(3).withTimeout(5),
      m_elevator.getSetpointCommand(ElevatorConstants.kL2Setpoint),
      new AutoAlignToReefLeft(m_db, LimelightConstants.kDistanceToReefLeft),  
      m_elevator.getSetpointCommand(ElevatorConstants.kL4Setpoint),          
      m_shooter.getShootCommand(0.28).withTimeout(2),
      m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint),
      m_db.resetEncodersCommand(),
      m_db.setGyroCommand(180)
    ));

    m_chooser.addOption("L4.alignToLeft-right", new SequentialCommandGroup(
      //moves to reef while elevator goes to L4
      m_db.resetEncodersCommand(),   //resets encoder positions
      m_db.resetGyroCommand(),   //resets gyroscope relative to the field
      m_db.getDriveCommand(3).withTimeout(5),
      m_elevator.getSetpointCommand(ElevatorConstants.kL2Setpoint),
      new AutoAlignToReefLeft(m_db, LimelightConstants.kDistanceToReefLeft),  
      m_elevator.getSetpointCommand(ElevatorConstants.kL4Setpoint),          
      m_shooter.getShootCommand(0.28).withTimeout(2),
      m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint),
      m_db.resetEncodersCommand(),
      m_db.setGyroCommand(240)
    ));

    m_chooser.addOption("L1-left", new SequentialCommandGroup(
      m_db.resetEncodersCommand(),
      m_db.resetGyroCommand(),
      new ParallelCommandGroup(
        m_elevator.getSetpointCommand(ElevatorConstants.kL1Setpoint),
        m_db.getDriveCommand(3).withTimeout(5)
      ),
      m_shooter.getShootCommand(0.2).withTimeout(5),
      m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint),
      m_db.resetEncodersCommand(),
      m_db.setGyroCommand(120)
    ));

    m_chooser.addOption("L1-center", new SequentialCommandGroup(
      m_db.resetEncodersCommand(),
      m_db.resetGyroCommand(),
      new ParallelCommandGroup(
        m_elevator.getSetpointCommand(ElevatorConstants.kL1Setpoint),
        m_db.getDriveCommand(3).withTimeout(5)
      ),
      m_shooter.getShootCommand(0.2).withTimeout(5),
      m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint),
      m_db.resetEncodersCommand(),
      m_db.setGyroCommand(180)
    ));

    m_chooser.addOption("L1-right", new SequentialCommandGroup(
      m_db.resetEncodersCommand(),
      m_db.resetGyroCommand(),
      new ParallelCommandGroup(
        m_elevator.getSetpointCommand(ElevatorConstants.kL1Setpoint),
        m_db.getDriveCommand(3).withTimeout(5)
      ),
      m_shooter.getShootCommand(0.2).withTimeout(5),
      m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint),
      m_db.resetEncodersCommand(),
      m_db.setGyroCommand(240)
    ));

    /* 
    m_chooser.addOption("TEST.L3.alignToLeft-center", new SequentialCommandGroup(
      //moves to reef while elevator goes to L4
      m_db.resetEncodersCommand(),   //resets encoder positions
      m_db.resetGyroCommand(),   //resets gyroscope relative to the field
      m_elevator.getSetpointCommand(ElevatorConstants.kL2Setpoint),
      new AutoAlignToReefLeft(m_db, LimelightConstants.kDistanceToReefLeft), 
      m_elevator.getSetpointCommand(ElevatorConstants.kL3Setpoint),          
      m_shooter.getShootCommand(0.17).withTimeout(2),
      m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint),
      m_db.resetEncodersCommand(),
      m_db.setGyroCommand(180)
    ));

    m_chooser.addOption("TEST.L4.alignToLeft-center", new SequentialCommandGroup(
      //moves to reef while elevator goes to L4
      m_db.resetEncodersCommand(),   //resets encoder positions
      m_db.resetGyroCommand(),   //resets gyroscope relative to the field
      m_elevator.getSetpointCommand(ElevatorConstants.kL2Setpoint),
      new AutoAlignToReefLeft(m_db, LimelightConstants.kDistanceToReefLeft),  
      m_elevator.getSetpointCommand(ElevatorConstants.kL4Setpoint),          
      m_shooter.getShootCommand(0.28).withTimeout(2),
      m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint),
      m_db.resetEncodersCommand(),
      m_db.setGyroCommand(180)
    ));
    */

    /* 
    m_chooser.addOption("L1-center", new SequentialCommandGroup(
      m_db.resetEncodersCommand(),
      m_db.resetGyroCommand(),
      new ParallelCommandGroup(
        m_elevator.getSetpointCommand(ElevatorConstants.kL1Setpoint),
        m_db.getDriveCommand(3).withTimeout(5)
      ),
      m_shooter.getShootCommand(0.2).withTimeout(5),
      m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint),
      m_db.resetEncodersCommand(),
      m_db.setGyroCommand(180)
    ));

    m_chooser.addOption("L1-left/right", new SequentialCommandGroup(
      m_db.resetEncodersCommand(),
      m_db.resetGyroCommand(),
      new ParallelCommandGroup(
        m_elevator.getSetpointCommand(ElevatorConstants.kL1Setpoint),
        m_db.getDriveCommand(6).withTimeout(6.5)
      ),
      m_shooter.getShootCommand(0.2).withTimeout(5),
      m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint)
    ));

    m_chooser.addOption("L2.alignToLeft-center", new SequentialCommandGroup(
      //moves to reef while elevator goes to L4
      m_db.resetEncodersCommand(),   //resets encoder positions
      m_db.resetGyroCommand(),   //resets gyroscope relative to the field
      new ParallelCommandGroup(  //runs multiple commands simultaneously 
        m_elevator.getSetpointCommand(ElevatorConstants.kL2Setpoint),   
        m_db.getDriveCommand(3.5).withTimeout(5)
        //drives forward set distance and brings elevator to specified setpoint
      ),
      new AutoAlignToReefLeft(m_db, LimelightConstants.kDistanceToReefLeft),            
      m_shooter.getShootCommand(0.17).withTimeout(2),
      m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint),
      m_db.resetEncodersCommand(),
      m_db.setGyroCommand(180)
    ));

    m_chooser.addOption("L4.alignToLeft-center", new SequentialCommandGroup(
      //moves to reef while elevator goes to L4
      m_db.resetEncodersCommand(),   //resets encoder positions
      m_db.resetGyroCommand(),   //resets gyroscope relative to the field
      new ParallelCommandGroup(  //runs multiple commands simultaneously 
        m_elevator.getSetpointCommand(ElevatorConstants.kL2Setpoint),   
        m_db.getDriveCommand(3.5).withTimeout(5)
        //drives forward set distance and brings elevator to specified setpoint
      ),
      new AutoAlignToReefLeft(m_db, LimelightConstants.kDistanceToReefLeft),
      m_elevator.getSetpointCommand(ElevatorConstants.kL4Setpoint),             
      m_shooter.getShootCommand(0.17).withTimeout(2),
      m_db.getReversedDriveCommand(0.2), //drive backwards to prevent collision with algae or reef             
      m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint),
      m_db.resetEncodersCommand(),
      m_db.setGyroCommand(180)
    ));
    
    m_chooser.addOption("L4.alignToLeft-left/right", new SequentialCommandGroup(
      //moves to reef while elevator goes to L4
      m_db.resetEncodersCommand(),
      m_db.resetGyroCommand(),
      new ParallelCommandGroup(
        m_elevator.getSetpointCommand(ElevatorConstants.kL2Setpoint),
        m_db.getDriveCommand(6).withTimeout(6.5)
      ),
      new RepeatCommand(
        new AutoAlignToReefLeft(m_db, LimelightConstants.kDistanceToReefLeft)
      ).withTimeout(3),
      m_elevator.getSetpointCommand(ElevatorConstants.kL4Setpoint),
      m_shooter.getShootCommand(0.5).withTimeout(2),
      m_db.getReversedDriveCommand(0.2), //drive backwards to prevent collision with algae or reef
      m_elevator.getSetpointCommand(ElevatorConstants.kBaseSetpoint)
    ));
    */
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}