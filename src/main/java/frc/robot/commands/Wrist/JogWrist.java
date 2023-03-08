// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;

public class JogWrist extends CommandBase {
  /** Creates a new JogWrist. */
  private WristSubsystem m_wrist;
  private DoubleSupplier m_speed;
  private boolean m_bypassLimit;
  private double throttle;
  private final SlewRateLimiter m_slewWrist = new SlewRateLimiter(WristConstants.JOG_SLEW_RATE, -10000, 0);

  public JogWrist(WristSubsystem wrist, DoubleSupplier speed, boolean bypassLimit) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_wrist = wrist;
    m_speed = speed;
    m_bypassLimit = bypassLimit;
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.resetFF = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  public void execute() {
    double throttleMultiplier = .5;

    throttle = MathUtil.applyDeadband(Math.abs(m_speed.getAsDouble()),
        WristConstants.kControllerDeadband)
        * Math.signum(m_speed.getAsDouble());

    throttle = Math.signum(throttle) * Math.pow(throttle, 2);

    double throttle_sl = m_slewWrist.calculate(throttle);

    throttle_sl *= throttleMultiplier;

    boolean allowUp = m_wrist.getAngleDegrees() <= WristConstants.MAX_ANGLE || m_bypassLimit;

    boolean allowDown = m_wrist.getAngleDegrees() >= WristConstants.MIN_ANGLE || m_bypassLimit;

    throttle_sl *= WristConstants.MAX_RADS_PER_SEC;

    m_wrist.commandRadPerSec = throttle_sl;

    double positionRadians = 0;//m_wrist.getAngleRadians();

    double angularVelocity = Units.degreesToRadians(throttle * WristConstants.MAX_RADS_PER_SEC);

    m_wrist.ff = m_wrist.m_armfeedforward.calculate(positionRadians, angularVelocity);

    if ((m_wrist.ff > 0 && allowUp) || (m_wrist.ff < 0 && allowDown))

      m_wrist.m_motor.setVoltage(m_wrist.ff);

    else

      m_wrist.m_motor.setVoltage(0);

    m_wrist.setControllerGoal(m_wrist.getAngleRadians());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.m_motor.setVoltage(0);
    m_wrist.setController(WristConstants.wristConstraints, m_wrist.getAngleRadians(), false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
