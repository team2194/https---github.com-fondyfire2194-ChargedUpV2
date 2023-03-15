// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PickupRoutines;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem.gamePiece;

public class GetPieceAtIntake extends CommandBase {
  /** Creates a new GetPieceAtIntake. */
  private IntakeSubsystem m_intake;

  private double m_speed;

  private double aveConeDist;
  private double aveCubeDist;
  private double aveAmps;

  public int cubeSenseThreshold = 300;

  public int coneSenseThreshold = 300;

  public double ampsThreshold = 20;

  LinearFilter cubeSensorFilter = LinearFilter.movingAverage(5);

  LinearFilter coneSensorFilter = LinearFilter.movingAverage(5);

  LinearFilter ampsFilter = LinearFilter.movingAverage(5);

  boolean ampsHigh;

  boolean cubeSeen;

  boolean coneSeen;

  public GetPieceAtIntake(IntakeSubsystem intake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

m_intake.moveManually(m_speed);

    aveAmps = ampsFilter.calculate(m_intake.getAmps());

    aveConeDist = coneSensorFilter.calculate(m_intake.getConeSensorDistance());

    aveCubeDist = cubeSensorFilter.calculate(m_intake.getCubeSensorDistance());

    cubeSeen = aveCubeDist < cubeSenseThreshold;

    coneSeen = aveConeDist < coneSenseThreshold;

    ampsHigh = aveAmps > ampsThreshold;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ampsHigh || cubeSeen || coneSeen;
  }
}
