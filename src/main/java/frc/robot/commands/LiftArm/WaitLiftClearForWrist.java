// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LiftArm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftArmSubsystem;

public class WaitLiftClearForWrist extends CommandBase {
  /** Creates a new WaitWristAtTarget. */
  private LiftArmSubsystem m_lift;

  private double m_clearAngle;

  public WaitLiftClearForWrist(LiftArmSubsystem lift, double clearAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lift = lift;

    m_clearAngle = clearAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_lift.getCanCoderPosition() > m_clearAngle;
  }
}
