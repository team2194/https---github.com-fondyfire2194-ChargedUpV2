// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LiftArm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetLiftGoal extends InstantCommand {
  private LiftArmSubsystem m_lift;
  private TrapezoidProfile.Constraints m_constraints;
  private double m_goalAngleRadians;

  public SetLiftGoal(LiftArmSubsystem lift, TrapezoidProfile.Constraints constraints, double goalAngleRadians) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_lift = lift;
    m_constraints = constraints;
    m_goalAngleRadians = goalAngleRadians;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lift.setController(m_constraints, m_goalAngleRadians, false);
  }
}
