// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveandBalanceRobot extends CommandBase {
  /** Creates a new BalanceRobot. */
  private DriveSubsystem m_drive;

  private boolean autoBalanceYMode;
  private double m_speed;
  boolean onChargeStation;
  private boolean autoBalanceXMode;

  private boolean balanceMode;
  static double kOffBalanceThresholdDegrees = 10.0;
  static double kOnBalanceThresholdDegrees = 5.0;

  public DriveandBalanceRobot(DriveSubsystem drive, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drive.drive(m_speed, 0, m_drive.getHeadingDegrees());

    if (Math.abs(m_drive.getGyroPitch()) > 5)

      onChargeStation = true;

    if (onChargeStation && Math.abs(m_drive.getGyroPitch()) < 5)
      balanceMode = true;

    double xAxisRate = 0;
    double yAxisRate = 0;
    double pitchAngleDegrees = m_drive.getGyroPitch();

    if (!autoBalanceXMode &&
        (Math.abs(pitchAngleDegrees) >= kOffBalanceThresholdDegrees)) {
      autoBalanceYMode = true;
    } else if (autoBalanceXMode &&
        (Math.abs(pitchAngleDegrees) <= kOnBalanceThresholdDegrees)) {
      autoBalanceYMode = false;
    }

    double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);

    xAxisRate = Math.sin(pitchAngleRadians) * -1;

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
