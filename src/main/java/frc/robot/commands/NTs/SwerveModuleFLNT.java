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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModuleSM;

public class SwerveModuleFLNT extends CommandBase {
  /** Creates a new extarmNT. */

  private SwerveModuleSM m_sm;

  public BooleanPublisher canokall;

  public BooleanPublisher modulefault;

  public BooleanPublisher turninposition;

  public BooleanPublisher driveopenloop;
  public BooleanPublisher fieldoriented;

  public DoublePublisher turnangle;
  public DoublePublisher turnvelocity;
  public DoublePublisher turnamps;
  public DoublePublisher turnout;
  public DoublePublisher turnset;

  public DoublePublisher driveposition;
  public DoublePublisher drivevelocity;
  public DoublePublisher driveamps;
  public DoublePublisher driveout;
  public DoublePublisher driveset;

  public StringPublisher firmwaredrive;
  public StringPublisher firmwareturn;

  public SwerveModuleFLNT( SwerveModuleSM sm) {

    m_sm = sm;

    // Use addRequirements() here to declare subsystem dependencies.
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable swerve = inst.getTable("swerve");

    NetworkTable frontleft = inst.getTable("swerve/frontleft");
    
    canokall = frontleft.getBooleanTopic("CANOK").publish();

    modulefault = frontleft.getBooleanTopic("MODULEFAULT").publish();

    turninposition = frontleft.getBooleanTopic("TURNINPOS").publish();

    firmwaredrive = frontleft.getStringTopic("FirmwareDrive").publish();

    firmwareturn = frontleft.getStringTopic("FirmwareTurn").publish();

    turnangle = frontleft.getDoubleTopic("TURNAMPS").publish();
    turnvelocity = frontleft.getDoubleTopic("TURNVEL").publish();
    turnout = frontleft.getDoubleTopic("TURNOUT").publish();
    turnset = frontleft.getDoubleTopic("TURNSET").publish();
    turnamps = frontleft.getDoubleTopic("TURNAMPS").publish();

    driveposition = frontleft.getDoubleTopic("DRIVEAMPS").publish();
    drivevelocity = frontleft.getDoubleTopic("DRIVEVEL").publish();
    driveout = frontleft.getDoubleTopic("DRIVEOUT").publish();
    driveset = frontleft.getDoubleTopic("DRIVESET").publish();
    driveamps = frontleft.getDoubleTopic("DRIVEAMPS").publish();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    canokall.set(m_sm.checkCAN());
    modulefault.set(m_sm.hasFault());
    turninposition.set(m_sm.turnInPosition(2));

    turnangle.set(m_sm.getTurnAngleDegs());
    turnvelocity.set(m_sm.getTurnVelocity());
    turnset.set(m_sm.angle);
    turnout.set(m_sm.getTurnAppliedOutput());
    turnamps.set(m_sm.getTurnCurrent());

    driveposition.set(m_sm.getDrivePosition());
    drivevelocity.set(m_sm.getDriveVelocity());
    driveset.set(m_sm.getDriveSpeedSetpoint());
    driveout.set(m_sm.getDriveAppliedOutput());
    driveamps.set(m_sm.getDriveCurrent());

    firmwareturn.set(m_sm.m_turnMotor.getFirmwareString());
    firmwaredrive.set(m_sm.m_driveMotor.getFirmwareString());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    canokall.close();

    modulefault.close();

    turninposition.close();

    driveopenloop.close();
    fieldoriented.close();

    turnangle.close();
    turnvelocity.close();
    turnamps.close();
    turnout.close();
    turnset.close();

    driveposition.close();
    drivevelocity.close();
    driveamps.close();
    driveout.close();
    driveset.close();

    firmwaredrive.close();
    firmwareturn.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
