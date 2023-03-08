// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ExtendArm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ExtendArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EndExtCommand extends InstantCommand {
  private ExtendArmSubsystem m_extend;

  public EndExtCommand(ExtendArmSubsystem extend) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_extend = extend;
    m_extend.endComm = true;
    addRequirements(m_extend);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
}
