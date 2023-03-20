// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LiftArm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftArmSubsystem;

public class WaitLiftAtTarget extends CommandBase {
  /** Creates a new WaitWristAtTarget. */
  private LiftArmSubsystem m_lift;
  private double m_startTime;
  private double m_atTargetTime;
  private double m_range;
  private double m_inRangeTime;

  public WaitLiftAtTarget(LiftArmSubsystem lift, double atTargetTime, double range) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lift = lift;
    m_atTargetTime = atTargetTime;
    m_range = range;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
    m_inRangeTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!m_lift.inRange(m_range))
      m_inRangeTime = 0;

    if (m_lift.inRange(m_range)) {
      m_inRangeTime = Timer.getFPGATimestamp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_lift.atTargetPosition()
        || m_inRangeTime != 0 && Timer.getFPGATimestamp() > m_inRangeTime + 1
        || Timer.getFPGATimestamp() > m_startTime + 2;
  }
}
