// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveDrivebaseConstants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopDrive extends Command {
  private final Swerve m_swerve;
  private final Supplier<Double> m_xSpeed;
  private final Supplier<Double> m_ySpeed;
  private final Supplier<Double> m_rot;
  private final Supplier<Boolean> m_toggleRobotRelative;  // robot drives robot relative when this returns true
  private final double m_multiplier;
  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(SwerveDrivebaseConstants.kSlewRateLimiter);    // limits acceleration of robot
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(SwerveDrivebaseConstants.kSlewRateLimiter);    // limits acceleration of robot
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(SwerveDrivebaseConstants.kSlewRateLimiter);  // limits acceleration of robot
  /** Creates a new TeleopDrive. */
  public TeleopDrive(final Swerve swerve, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rot, Supplier<Boolean> toggleRobotRelative, final double multiplier) {
    m_swerve = swerve;
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_rot = rot;
    m_toggleRobotRelative = toggleRobotRelative;
    m_multiplier = MathUtil.clamp(multiplier, 0.0, 1.0);  // multiplier should be unsigned [0, 1] to avoid exceeding maximum speed
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Negative sign applied to joystick values per the differing coordinate systems between robot and joysticks.
     * @link https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
     */
    m_swerve.drive(calcDeliveredStickSpeed(-m_xSpeed.get(), m_xLimiter) * m_multiplier * SwerveDrivebaseConstants.kMaxMetersPerSecond,
                   calcDeliveredStickSpeed(-m_ySpeed.get(), m_yLimiter) * m_multiplier * SwerveDrivebaseConstants.kMaxMetersPerSecond,
                   calcDeliveredStickSpeed(-m_rot.get(), m_rotLimiter)  * m_multiplier * SwerveDrivebaseConstants.kMaxAngularSpeed,
                   !m_toggleRobotRelative.get());   // robot relative is true means field relative is false
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(0.0, 0.0, 0.0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /** Calculates the joystick value we deliver to the drive method using both a slew rate limiter
   * to limit acceleration, as well as squaring the joystick values to have finer control at lower
   * speeds.  To see a graphical representation of this, compare the graphs of y = x from -1 to 1
   * and y = sgn(x) * x^2 from -1 to 1.  The latter is flatter (and lower) at lower speeds, allowing
   * for finer control.  Note that you could cube the values for even finer control.
   * 
   * @param   joystick  The input joystick value
   * @param   limiter   The slew rate limiter for this joystick
   * 
   * @return  The output, calculated joystick value
   * 
   * {@link}  https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#squaring-inputs
   * {@link}  https://www.desmos.com/calculator/eztstjacns
   */
  private double calcDeliveredStickSpeed(final double joystick, final SlewRateLimiter limiter) {
    final double deadbandValue = MathUtil.applyDeadband(joystick, ControllerConstants.kDeadband);
    final double squaredValue = Math.copySign(deadbandValue * deadbandValue, deadbandValue);
    return limiter.calculate(squaredValue);
  }
}
