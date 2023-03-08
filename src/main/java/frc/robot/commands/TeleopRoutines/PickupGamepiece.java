// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.presetIntakeSpeeds;

public class PickupGamepiece extends CommandBase {
  /** Creates a new DeliverGamepiece. */
  private IntakeSubsystem m_intake;
  private boolean m_cube;

  public PickupGamepiece(IntakeSubsystem intake, boolean cube) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_cube = cube;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_cube) {

      m_intake.moveManually(presetIntakeSpeeds.PICKUP_CUBE.getSpeed());

    } else {

      m_intake.moveManually(presetIntakeSpeeds.PICKUP_CONE.getSpeed());

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return m_cube && m_intake.cubePresent

        || !m_cube && m_intake.conePresent;
  }
}
