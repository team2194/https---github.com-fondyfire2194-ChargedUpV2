// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LiftArm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftArmSubsystem;

public class RunLiftArmProfile extends CommandBase {
  /** Creates a new RunExtArmProfile. */
  private LiftArmSubsystem m_lift;
  private TrapezoidProfile.Constraints m_constraints;
  private double m_goalAngle;
  private int loopctr;

  public RunLiftArmProfile(LiftArmSubsystem lift, TrapezoidProfile.Constraints constraints, double goalAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lift = lift;
    m_constraints=constraints;
    m_goalAngle = goalAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lift.setControllerConstraints(m_constraints);
    m_lift.setGoal(m_goalAngle);
    loopctr = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loopctr++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return loopctr > 10 && m_lift.atTargetPosition();
  }
}
