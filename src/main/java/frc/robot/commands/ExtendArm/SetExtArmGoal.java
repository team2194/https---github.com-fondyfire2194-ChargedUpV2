// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ExtendArm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.subsystems.ExtendArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetExtArmGoal extends CommandBase {
  private ExtendArmSubsystem m_ext;
  private TrapezoidProfile.Constraints m_constraints;
  private double m_goalDist;
  int tstctr;

  public SetExtArmGoal(ExtendArmSubsystem ext, TrapezoidProfile.Constraints constraints, double goalDist) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_ext = ext;
    m_constraints = constraints;
    m_goalDist = goalDist;
  }

  public SetExtArmGoal(ExtendArmSubsystem ext, double goalDist) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_ext = ext;
    m_constraints = ExtendArmConstants.extendArmConstraints;
    m_goalDist = goalDist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ext.setController(m_constraints, m_goalDist, false);
    m_ext.deliverDistance = m_goalDist;
  }

  @Override
  public void execute() {
    m_ext.setController(m_constraints, m_goalDist, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ext.goalInches == m_goalDist;
  }

}
