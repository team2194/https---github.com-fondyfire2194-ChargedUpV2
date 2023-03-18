// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LiftArm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.subsystems.LiftArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetLiftGoal extends CommandBase {
  private LiftArmSubsystem m_lift;
  private TrapezoidProfile.Constraints m_constraints;
  private double m_goalInches;

  public SetLiftGoal(LiftArmSubsystem lift, TrapezoidProfile.Constraints constraints, double goalInches) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_lift = lift;
    m_constraints = constraints;
    m_goalInches=goalInches;
    
  }

  public SetLiftGoal(LiftArmSubsystem lift, double goalInches) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_lift = lift;
    m_constraints = LiftArmConstants.liftArmFastConstraints;
    m_goalInches = goalInches;
    //addRequirements(m_lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lift.setController(m_constraints, m_goalInches, false);
    m_lift.goalInches = m_goalInches;
  }

  @Override
  public void execute() {
   // m_lift.setController(m_constraints, m_goalInches, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
