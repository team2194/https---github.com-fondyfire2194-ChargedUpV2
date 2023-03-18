// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.Test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Will drive robot onto charge station from either side.
 * Robot should be squared up with station before starting
 * Robot will detect it is on ramp using the gyro pitch
 * then drive forward until is is level again.
 * Drive will use gyro yaw to keep straight
 * 
 */
public class DriveOnChargeStation extends CommandBase {
  /** Creates a new DriveOnChargeStation. */

  autoBalance ab = new autoBalance();

  DriveSubsystem m_drive;

  double m_value;

  public DriveOnChargeStation(DriveSubsystem drive, double value) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed = ab.autoBalanceRoutine() * m_value;

    m_drive.drive(speed, 0, 0);
  }

  ;

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
