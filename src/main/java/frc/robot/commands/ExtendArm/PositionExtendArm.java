// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ExtendArm;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendArmSubsystem;

public class PositionExtendArm extends CommandBase {
  /** Creates a new PositionExtendArm. */
  private ExtendArmSubsystem m_extend;
  private double m_speed;

  public PositionExtendArm(ExtendArmSubsystem extend, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_extend = extend;

    m_speed = speed;
    addRequirements(m_extend);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
   

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_extend.m_posnController.setReference(m_extend.positionTarget, ControlType.kPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
