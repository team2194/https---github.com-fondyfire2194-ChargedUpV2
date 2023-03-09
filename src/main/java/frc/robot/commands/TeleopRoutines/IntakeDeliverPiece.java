// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.presetIntakeSpeeds;

public class IntakeDeliverPiece extends CommandBase {
  /** Creates a new IntakeDeliverPiece. */
  private IntakeSubsystem m_intake;
  private boolean noConeAtStart;
  private double m_startTime;
  private double deliverTime = 1;

  public IntakeDeliverPiece(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    noConeAtStart = !m_intake.cubePresent && !m_intake.conePresent;

    m_startTime = Timer.getFPGATimestamp();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!m_intake.conePresent) {
      m_intake.moveManually(presetIntakeSpeeds.DELIVER_CONE.getSpeed());
    }

    if (!m_intake.cubePresent) {
      m_intake.moveManually(presetIntakeSpeeds.DELIVER_CUBE.getSpeed());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return noConeAtStart || Timer.getFPGATimestamp() > m_startTime + deliverTime;
  }
}
