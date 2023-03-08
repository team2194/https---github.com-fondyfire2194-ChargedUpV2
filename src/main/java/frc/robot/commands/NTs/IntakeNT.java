// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NTs;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;

public class IntakeNT extends CommandBase {
  /** Creates a new TurnArmNT. */
  private IntakeSubsystem m_intake;

  public BooleanPublisher canok;
  public BooleanPublisher isstopped;
  public BooleanPublisher atSpeed;
  public BooleanPublisher cubeSensorOK;
  public BooleanPublisher coneSensorOK;

  public DoublePublisher velocity;
  public DoublePublisher amps;
  public DoublePublisher out;
  public DoublePublisher coneSensorDistance;
  public DoublePublisher cubeSensorDistance;

  public StringPublisher firmware;

  public IntakeNT(IntakeSubsystem intK) {

    m_intake = intK;
    // Use addRequirements() here to declare subsystem dependencies.
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable intake = inst.getTable("intake");

    // publish to the topic in "datatable" called "Out"
    canok = intake.getBooleanTopic("CANOK").publish();

    atSpeed = intake.getBooleanTopic("INPOSITION").publish();
    isstopped = intake.getBooleanTopic("STOPPED").publish();
    cubeSensorOK= intake.getBooleanTopic("CUBESENSOROK").publish();
    coneSensorOK= intake.getBooleanTopic("CONESENSOROK").publish();

    velocity = intake.getDoubleTopic("Velocity").publish();
    amps = intake.getDoubleTopic("Current Amps").publish();
    out = intake.getDoubleTopic("MotorOut").publish();
    coneSensorDistance = intake.getDoubleTopic("ConeSensorDist").publish();
    cubeSensorDistance = intake.getDoubleTopic("CubeSensorDist").publish();
    

    firmware = intake.getStringTopic("Firmware").publish();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    canok.set(m_intake.checkCANOK());

    atSpeed.set(m_intake.getIntakeAtSpeed());
    isstopped.set(m_intake.isStopped());
    coneSensorOK.set(m_intake.coneSensorOK);
    cubeSensorOK.set(m_intake.cubeSensorOK);

    velocity.set(m_intake.getRPM());
    amps.set(m_intake.getAmps());
    out.set(m_intake.getAppliedOutput());
    coneSensorDistance.set(m_intake.getConeSensorDistance());
    cubeSensorDistance.set(m_intake.getCubeSensorDistance());



    firmware.set(m_intake.getFirmware());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    canok.close();

    atSpeed.close();

    isstopped.close();

    velocity.close();

    amps.close();

    out.close();

    firmware.close();

    coneSensorDistance.close();

    coneSensorOK.close();

    cubeSensorDistance.close();

    cubeSensorOK.close();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
