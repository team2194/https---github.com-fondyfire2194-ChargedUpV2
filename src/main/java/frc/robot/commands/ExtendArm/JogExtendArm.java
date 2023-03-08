// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ExtendArm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.subsystems.ExtendArmSubsystem;

public class JogExtendArm extends CommandBase {
  /** Creates a new JogArm. */
  private ExtendArmSubsystem m_ext;
  private DoubleSupplier m_speed;
  private boolean m_bypassLimit;
  private double throttle;
  private final SlewRateLimiter m_slewExt = new SlewRateLimiter(ExtendArmConstants.JOG_SLEW_RATE, -10000, 0);

  public JogExtendArm(ExtendArmSubsystem ext, DoubleSupplier speed, boolean bypassLimit) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ext = ext;
    m_speed = speed;
    m_bypassLimit = bypassLimit;
    addRequirements(m_ext);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ext.resetFF = true;
    m_ext.firstUp = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double throttleMultiplier = .5;

    throttle = MathUtil.applyDeadband(Math.abs(m_speed.getAsDouble()),
        ExtendArmConstants.kControllerDeadband)
        * Math.signum(m_speed.getAsDouble());

    throttle = Math.signum(throttle) * Math.pow(throttle, 2);

    double throttle_sl = m_slewExt.calculate(throttle);

    throttle_sl *= throttleMultiplier;

    boolean allowOut = m_ext.getPositionInches() <= ExtendArmConstants.MAX_POSITION || m_bypassLimit;

    boolean allowIn = m_ext.getPositionInches() >= ExtendArmConstants.MIN_POSITION || m_bypassLimit;

    throttle_sl *= ExtendArmConstants.MAX_RATE_INCHES_PER_SEC;

    m_ext.commandIPS = throttle_sl;

    m_ext.ff = m_ext.m_feedforward.calculate(throttle_sl);

    if (m_ext.ff > 0 && allowOut || m_ext.ff < 0 && allowIn)

      m_ext.m_motor.setVoltage(m_ext.ff);

    else

      m_ext.m_motor.setVoltage(0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_ext.m_motor.setVoltage(0);

    new PositionProfileExtendArm(m_ext, ExtendArmConstants.extendArmConstraints, m_ext.getPositionInches()).schedule();

    // m_ext.setGoal(m_ext.getPositionInches());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
