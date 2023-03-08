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
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.subsystems.ExtendArmSubsystem;

public class ExtendArmNT extends CommandBase {
  /** Creates a new extarmNT. */
  private ExtendArmSubsystem m_ext;

  public BooleanPublisher canok;
  public BooleanPublisher plusswlim;
  public BooleanPublisher minusswlim;
  public BooleanPublisher inposition;
  public BooleanPublisher isstopped;

  public DoublePublisher position;
  public DoublePublisher velocity;
  public DoublePublisher amps;
  public DoublePublisher out;
  public DoublePublisher targetangle;
  public DoublePublisher ff;
  public DoublePublisher kv;
  public DoublePublisher ks;
  public DoublePublisher commandIPS;
  

  public IntegerPublisher faults;

  public StringPublisher firmware;

  public ExtendArmNT(ExtendArmSubsystem ext) {

    m_ext = ext;
    // Use addRequirements() here to declare subsystem dependencies.
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable extarm = inst.getTable("extarm");

    // publish to the topic in "datatable" called "Out"
    canok = extarm.getBooleanTopic("CANOK").publish();
    plusswlim = extarm.getBooleanTopic("SWLIM+").publish();
    minusswlim = extarm.getBooleanTopic("SWLIM-").publish();
    inposition = extarm.getBooleanTopic("INPOSITION").publish();
    isstopped = extarm.getBooleanTopic("STOPPED").publish();

    position = extarm.getDoubleTopic("Position").publish();
    targetangle = extarm.getDoubleTopic("TargetPosition").publish();
    velocity = extarm.getDoubleTopic("Velocity").publish();
    amps = extarm.getDoubleTopic("Current Amps").publish();
    out = extarm.getDoubleTopic("MotorOut").publish();
    ff = extarm.getDoubleTopic("Feedforward").publish();
    kv = extarm.getDoubleTopic("KV").publish();
    ks = extarm.getDoubleTopic("KS").publish();
    commandIPS = extarm.getDoubleTopic("CommandIPS").publish();
    

    faults = extarm.getIntegerTopic("FAULTS").publish();

    firmware = extarm.getStringTopic("Firmware").publish();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    canok.set(m_ext.checkCANOK());
    plusswlim.set(m_ext.onPlusSoftwareLimit());
    minusswlim.set(m_ext.onMinusSoftwareLimit());
    inposition.set(m_ext.atTargetPosition());
    isstopped.set(m_ext.isStopped());

    position.set(m_ext.positionInches);
    targetangle.set(m_ext.goalInches);
    velocity.set(m_ext.inchespersec);
    amps.set(m_ext.amps);
    out.set(m_ext.appliedOutput);
    ff.set(m_ext.ff);
    kv.set(ExtendArmConstants.kvExtArmVoltSecondsPerInch);
    ks.set(ExtendArmConstants.ksExtArmVolts);
    commandIPS.set(m_ext.commandIPS);

    faults.set(m_ext.getFaults());

    firmware.set(m_ext.getFirmware());

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

    firmware.close();

    faults.close();

    ff.close();

    kv.close();
    
    ks.close();

    commandIPS.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
