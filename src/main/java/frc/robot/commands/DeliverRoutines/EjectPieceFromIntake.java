// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeliverRoutines;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem.robotPiece;

public class EjectPieceFromIntake extends CommandBase {
  /** Creates a new GetPieceAtIntake. */
  private IntakeSubsystem m_intake;

  private double m_speed = .7;

  private robotPiece m_type;

  private double aveConeDist;
  private double aveCubeDist;
  private double aveAmps;

  public int cubeSenseThreshold = 300;

  public int coneSenseThreshold = 300;

  public double ampsThreshold = 20;

 LinearFilter coneSensorFilter = LinearFilter.movingAverage(5);

  LinearFilter ampsFilter = LinearFilter.movingAverage(5);

  boolean ampsHigh;

  boolean noCubeSeen;

  boolean noConeSeen;

  public EjectPieceFromIntake(IntakeSubsystem intake) {
    m_intake = intake;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (m_intake.piece == robotPiece.CUBE)

      m_speed *= -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_intake.moveManually(m_speed);

    aveConeDist = coneSensorFilter.calculate(m_intake.getConeSensorDistance());

    noCubeSeen = aveCubeDist > cubeSenseThreshold;

    noConeSeen = aveConeDist > coneSenseThreshold;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return noCubeSeen && noConeSeen;
  }
}
