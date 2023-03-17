// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RaiseLowerWrist extends InstantCommand {
  private WristSubsystem m_wrist;
  private boolean m_direction;
  private double inc = 2;

  public RaiseLowerWrist(WristSubsystem wrist, boolean direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_wrist = wrist;
    m_direction = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_direction)
      inc *= -1;
    double temp = m_wrist.goalAngleRadians + inc;
    m_wrist.setController(WristConstants.wristConstraints, temp, false);

  }
}
