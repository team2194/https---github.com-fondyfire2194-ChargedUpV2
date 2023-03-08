// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LiftArm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.subsystems.LiftArmSubsystem;

public class JogLiftArm extends CommandBase {
  /** Creates a new JogArm. */
  private LiftArmSubsystem m_lift;
  private DoubleSupplier m_speed;
  private double throttle;
  private double throttleMultiplier = .25;

  private final SlewRateLimiter m_slewLift = new SlewRateLimiter(LiftArmConstants.JOG_SLEW_RATE, -10000, 0);

  public JogLiftArm(LiftArmSubsystem lift, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lift = lift;
    m_speed = speed;
    addRequirements(m_lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lift.resetFF = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    throttle = MathUtil.applyDeadband(Math.abs(m_speed.getAsDouble()),
        LiftArmConstants.kControllerDeadband)
        * Math.signum(m_speed.getAsDouble());

    throttle = Math.signum(throttle) * Math.pow(throttle, 2);

    double throttle_sl = m_slewLift.calculate(throttle);

    boolean allowUp = m_lift.getCanCoderPosition() <= LiftArmConstants.MAX_ANGLE;

    boolean allowDown = m_lift.getCanCoderPosition() >= LiftArmConstants.MIN_ANGLE;

    throttle_sl *= throttleMultiplier;

    throttle_sl *= LiftArmConstants.MAX_RATE_INCHES_PER_SEC;

    m_lift.commandIPS = throttle_sl;

    double positionRadians = 0;

    m_lift.ff = m_lift.m_armFeedforward.calculate(positionRadians, throttle_sl);

    if (m_lift.ff > 0 & allowUp || m_lift.ff < 0 && allowDown) {

      m_lift.m_motor.setVoltage(m_lift.ff);
    }

    else {

      m_lift.m_motor.setVoltage(0);
    }

    m_lift.setGoal(m_lift.getPositionInches());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lift.m_motor.setVoltage(0);
    m_lift.setGoal(m_lift.getPositionInches());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
