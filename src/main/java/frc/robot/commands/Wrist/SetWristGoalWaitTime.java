// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetWristGoalWaitTime extends CommandBase {
  private WristSubsystem m_wrist;

  private TrapezoidProfile.Constraints m_constraints;
  private double m_goalAngleRadians;

  private boolean goalset;
  private double m_startTime;
  private double m_time;

  public SetWristGoalWaitTime(WristSubsystem wrist, TrapezoidProfile.Constraints constraints,
      double goalAngleRadians, double time) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_wrist = wrist;

    m_constraints = constraints;
    m_goalAngleRadians = goalAngleRadians;
    m_time = time;
  }

  public SetWristGoalWaitTime(WristSubsystem wrist, double goalAngleRadians,
      double time) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_wrist = wrist;
    m_constraints = WristConstants.wristFastConstraints;
    m_goalAngleRadians = goalAngleRadians;
    m_time = time;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goalset = false;
    m_startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() > m_startTime + m_time) {
      m_wrist.setController(m_constraints, m_goalAngleRadians, false);
      m_wrist.goalAngleRadians = m_goalAngleRadians;
      goalset = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return goalset;
  }
}
