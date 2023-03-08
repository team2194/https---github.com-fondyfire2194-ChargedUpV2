// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ExtendArm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.subsystems.ExtendArmSubsystem;

public class RunExtArmProfile extends CommandBase {
  /** Creates a new RunExtArmProfile. */
  private ExtendArmSubsystem m_ext;
  private double m_goalinches;
  private TrapezoidProfile.Constraints m_constraints;
  private int loopctr;

  public RunExtArmProfile(ExtendArmSubsystem ext, TrapezoidProfile.Constraints constraints, double goalInches) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ext = ext;
    m_goalinches = goalInches;
    m_constraints = constraints;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ext.setControllerConstraints(m_constraints);
    m_ext.setGoal(m_goalinches);
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
    m_ext.setControllerConstraints(ExtendArmConstants.extendArmConstraints);
    m_ext.setGoal(m_ext.getPositionInches());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return loopctr > 10 && m_ext.atTargetPosition() && m_ext.isStopped();
  }
}
