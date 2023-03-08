// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ExtendArm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetExtArmGoal extends InstantCommand {
  private ExtendArmSubsystem m_ext;
  private TrapezoidProfile.Constraints m_constraints;
  private double m_goalDist;

  public SetExtArmGoal(ExtendArmSubsystem ext, TrapezoidProfile.Constraints constraints, double goalDist) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_ext = ext;
    m_constraints = constraints;
    m_goalDist = goalDist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ext.setController(m_constraints, m_goalDist, false);
  }
}
