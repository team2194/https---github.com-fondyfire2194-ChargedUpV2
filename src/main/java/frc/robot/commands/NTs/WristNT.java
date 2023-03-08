// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NTs;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;

public class WristNT extends CommandBase {
  /** Creates a new wristNT. */
  private WristSubsystem m_wrist;

  public BooleanPublisher canok;
  public BooleanPublisher plusswlim;
  public BooleanPublisher minusswlim;
  public BooleanPublisher plushwlim;
  public BooleanPublisher minushwlim;
  public BooleanPublisher inposition;
  public BooleanPublisher isstopped;
  public BooleanPublisher resetdone;;

  public DoublePublisher position;
  public DoublePublisher velocity;
  public DoublePublisher amps;
  public DoublePublisher out;
  public DoublePublisher targetangle;
  public DoublePublisher ff;
  public DoublePublisher kv;
  public DoublePublisher ks;
  public DoublePublisher commandDPS;

  public IntegerPublisher faults;

  public StringPublisher firmware;

  public WristNT(WristSubsystem wristS) {

    m_wrist = wristS;
    // Use addRequirements() here to declare subsystem dependencies.
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable wrist = inst.getTable("wrist");

    // publish to the topic in "datatable" called "Out"
    canok = wrist.getBooleanTopic("CANOK").publish();
    plusswlim = wrist.getBooleanTopic("SWLIM+").publish();
    minusswlim = wrist.getBooleanTopic("SWLIM-").publish();
    inposition = wrist.getBooleanTopic("INPOSITION").publish();
    isstopped = wrist.getBooleanTopic("STOPPED").publish();

    position = wrist.getDoubleTopic("Position").publish();
    targetangle = wrist.getDoubleTopic("TargetAngle").publish();
    velocity = wrist.getDoubleTopic("Velocity").publish();
    amps = wrist.getDoubleTopic("Current Amps").publish();
    out = wrist.getDoubleTopic("MotorOut").publish();
    ff = wrist.getDoubleTopic("Feedforward").publish();
    kv = wrist.getDoubleTopic("KV").publish();
    ks = wrist.getDoubleTopic("KS").publish();
    commandDPS = wrist.getDoubleTopic("CommandDPS").publish();

    faults = wrist.getIntegerTopic("FAULTS").publish();

    firmware = wrist.getStringTopic("Firmware").publish();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    canok.set(m_wrist.checkCANOK());
    plusswlim.set(m_wrist.onPlusSoftwareLimit());
    minusswlim.set(m_wrist.onMinusSoftwareLimit());

    inposition.set(m_wrist.atTargetAngle());
    isstopped.set(m_wrist.isStopped());

    position.set(m_wrist.positionDegrees);
    targetangle.set(m_wrist.endpointDegrees);
    velocity.set(m_wrist.degreespersec);
    amps.set(m_wrist.amps);
    out.set(m_wrist.appliedOutput);
    ff.set(m_wrist.ff);
    kv.set(WristConstants.kVVoltSecondPerRad);
    ks.set(WristConstants.ksVolts);
    commandDPS.set(m_wrist.commandDPS);

    faults.set(m_wrist.getFaults());

    firmware.set(m_wrist.getFirmware());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    canok.close();

    plusswlim.close();

    minusswlim.close();

    inposition.close();

    isstopped.close();

    position.close();

    velocity.close();

    amps.close();

    out.close();

    targetangle.close();

    faults.close();

    ff.close();

    kv.close();

    ks.close();

    commandDPS.close();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
