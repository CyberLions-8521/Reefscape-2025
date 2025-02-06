// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivebase;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final CommandXboxController m_gamepad = new CommandXboxController(0);
  private final Drivebase m_db = new Drivebase();

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  private double limelight_aim_proportional()
  {    
    // Check to see if there is a valid target
    if ( !LimelightHelpers.getTV("limelight") ) return 0.0;

    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= DriveConstants.kMaxAngularSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  private double limelight_range_proportional()
  {    
    // Check to see if there is a valid target
    if ( !LimelightHelpers.getTV("limelight") ) return 0.0;

    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    // Default command using gamepad joystick axes
    m_db.setDefaultCommand(
        getDriveCommand(
            m_gamepad::getLeftY,
            m_gamepad::getLeftX,
            m_gamepad::getRightX,
            true
        )
    );

    // Command when A button is pressed
    // Right now it uses fixed 0 values for the x and y speed, and uses the limelight
    // to get the angular velocity.
    //
    // Hopefully this will rotate and turn the robot to face the target.
    m_gamepad.a().onTrue(
        getDriveCommand(
            () -> 0.0,
            () -> 0.0,
            () -> limelight_aim_proportional(),
            true
        )
    );

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  /**
   * Creates a command that runs the drive method from the Drivebase object
   * @param xSpeed  A method reference or lambda that returns the speed of the chassis in the x-direction
   * @param ySpeed  A method reference or lambda that returns the speed of the chassis in the y-direction
   * @param rot     A method reference or lambda that returns the speed of the chassis in the z-rotational direction
   * @param fieldRelative   Whether or not you are driving the robot with speeds relative to the field
   * @return  The drive command
   */
  public Command getDriveCommand(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rot, boolean fieldRelative) {
    return new RunCommand(
      () -> m_db.drive(
        -MathUtil.applyDeadband(xSpeed.get(), ControllerConstants.kDeadband) * DriveConstants.kMaxSpeedMetersPerSecond,
        -MathUtil.applyDeadband(ySpeed.get(), ControllerConstants.kDeadband) * DriveConstants.kMaxSpeedMetersPerSecond,
        -MathUtil.applyDeadband(rot.get(), ControllerConstants.kDeadband) * DriveConstants.kMaxAngularSpeed,
        fieldRelative),
      m_db);
  }
}
