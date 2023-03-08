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
import frc.robot.Constants.LiftArmConstants;
import frc.robot.subsystems.LiftArmSubsystem;

public class LiftArmNT extends CommandBase {
  /** Creates a new LiftArmNT. */
  private LiftArmSubsystem m_lift;

  public BooleanPublisher canok;
  

  public BooleanPublisher inposition;
  public BooleanPublisher isstopped;

  public DoublePublisher position;
  public DoublePublisher cancoderposition;

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

  public LiftArmNT(LiftArmSubsystem turn) {

    m_lift = turn;
    // Use addRequirements() here to declare subsystem dependencies.
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable liftarm = inst.getTable("liftarm");

    // publish to the topic in "datatable" called "Out"
    canok = liftarm.getBooleanTopic("CANOK").publish();
    

    inposition = liftarm.getBooleanTopic("INPOSITION").publish();
    isstopped = liftarm.getBooleanTopic("STOPPED").publish();

    position = liftarm.getDoubleTopic("Position").publish();
    cancoderposition = liftarm.getDoubleTopic("CancoderPosition").publish();
    targetangle = liftarm.getDoubleTopic("TargetAngle").publish();
    velocity = liftarm.getDoubleTopic("Velocity").publish();
    amps = liftarm.getDoubleTopic("Current Amps").publish();
    out = liftarm.getDoubleTopic("MotorOut").publish();
    ff = liftarm.getDoubleTopic("Feedforward").publish();
    kv = liftarm.getDoubleTopic("KV").publish();
    ks = liftarm.getDoubleTopic("KS").publish();
    commandIPS = liftarm.getDoubleTopic("CommaandIPS").publish();
   
    faults = liftarm.getIntegerTopic("FAULTS").publish();

    firmware = liftarm.getStringTopic("Firmware").publish();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    canok.set(m_lift.checkCANOK());
  
    inposition.set(m_lift.atTargetPosition());
    isstopped.set(m_lift.isStopped());

    position.set(m_lift.getCanCoderRadians());
    cancoderposition.set(m_lift.cancoderPosition);
    targetangle.set(m_lift.endpointInches);
    velocity.set(m_lift.getCanCoderRateRadsPerSec());
    amps.set(m_lift.amps);
    out.set(m_lift.appliedOutput);
    ff.set(m_lift.ff);
    kv.set(LiftArmConstants.kVVoltSecondPerInch);
    ks.set(LiftArmConstants.kSVolts);
    commandIPS.set(m_lift.commandIPS);

    faults.set(m_lift.getFaults());
    firmware.set(m_lift.getFirmware());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    canok.close();

    inposition.close();

    isstopped.close();

    position.close();

    cancoderposition.close();

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
