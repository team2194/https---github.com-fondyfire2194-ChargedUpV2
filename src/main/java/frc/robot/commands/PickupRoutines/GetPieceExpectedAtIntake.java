// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PickupRoutines;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem.gamePiece;
import frc.robot.subsystems.GameHandlerSubsystem.robotPiece;

public class GetPieceExpectedAtIntake extends CommandBase {
  /** Creates a new GetPieceAtIntake. */
  private IntakeSubsystem m_intake;
  private GameHandlerSubsystem m_ghs;

  private gamePiece m_type;

  private double aveConeDist;
  private double aveCubeDist;
  private double aveAmps;

  public int cubeSenseThreshold = 300;

  public int coneSenseThreshold = 300;

  public double ampsThreshold = 20;

  LinearFilter cubeSensorFilter = LinearFilter.movingAverage(25);

  LinearFilter coneSensorFilter = LinearFilter.movingAverage(25);

  boolean cubeSeen;

  boolean coneSeen;

  double useSpeed;

  private double m_startTimeCone;
  private double m_startTimeCube;

  public GetPieceExpectedAtIntake(IntakeSubsystem intake, GameHandlerSubsystem ghs) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_ghs = ghs;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_startTimeCone = 0;
    m_startTimeCube = 0;
    useSpeed = .6;
    if (m_ghs.getGamePiecetype() == gamePiece.CONE)

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

    if (!cubeSeen)
      m_startTimeCube = 0;

    if (cubeSeen && m_startTimeCube == 0) {
      m_startTimeCube = Timer.getFPGATimestamp();
    }

    if (!coneSeen)
      m_startTimeCone = 0;

    if (coneSeen && m_startTimeCone == 0) {
      m_startTimeCone = Timer.getFPGATimestamp();
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
    return cubeSeen && m_startTimeCube != 0 && Timer.getFPGATimestamp() > m_startTimeCube + 1

        || coneSeen && m_startTimeCone != 0 && Timer.getFPGATimestamp() > m_startTimeCone + 1;
  }
}
