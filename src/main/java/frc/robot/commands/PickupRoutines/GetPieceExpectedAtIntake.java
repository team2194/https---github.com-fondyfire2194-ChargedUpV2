// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PickupRoutines;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem.gamePiece;

public class GetPieceExpectedAtIntake extends CommandBase {
  /** Creates a new GetPieceAtIntake. */
  private IntakeSubsystem m_intake;

  private gamePiece m_type;

  private double aveConeDist;
  private double aveCubeDist;
  private double aveAmps;

  public int cubeSenseThreshold = 300;

  public int coneSenseThreshold = 300;

  public double ampsThreshold = 20;

  LinearFilter cubeSensorFilter = LinearFilter.movingAverage(5);

  LinearFilter coneSensorFilter = LinearFilter.movingAverage(5);

  boolean cubeSeen;

  boolean coneSeen;

  double useSpeed = .6;

  public GetPieceExpectedAtIntake(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (m_intake.expectedPieceType == gamePiece.CONE)
      useSpeed *= -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_intake.moveManually(useSpeed);

    aveConeDist = coneSensorFilter.calculate(m_intake.getConeSensorDistance());

    aveCubeDist = cubeSensorFilter.calculate(m_intake.getCubeSensorDistance());

    cubeSeen = aveCubeDist < cubeSenseThreshold;

    coneSeen = aveConeDist < coneSenseThreshold;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cubeSeen || coneSeen;
  }
}