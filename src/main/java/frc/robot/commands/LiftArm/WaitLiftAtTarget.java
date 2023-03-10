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

  public WaitLiftAtTarget(LiftArmSubsystem lift, double atTargetTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lift=lift;
    m_atTargetTime=atTargetTime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_lift.atTargetAngle() && m_startTime == 0) {
      m_startTime = Timer.getFPGATimestamp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() > m_startTime + m_atTargetTime;
  }
}
