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


public class DriveNT extends CommandBase {
  /** Creates a new swerveNT. */
  private DriveSubsystem m_drive;

  public BooleanPublisher openloop;
  public BooleanPublisher fieldOriented;
  
  public DoublePublisher X;
  public DoublePublisher Y;
  public DoublePublisher heading;
  

  public DriveNT(DriveSubsystem drive) {

    m_drive=drive;
    // Use addRequirements() here to declare subsystem dependencies.
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable swerve = inst.getTable("swerve/drive");

    // publish to the topic in "datatable" called "Out"
    openloop = swerve.getBooleanTopic("OPENLOOP").publish();
    fieldOriented = swerve.getBooleanTopic("FIELDORIENTED").publish();
    

    X = swerve.getDoubleTopic("X").publish();
    Y = swerve.getDoubleTopic("Y").publish();
  heading = swerve.getDoubleTopic("HEADING").publish();
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    openloop.set(m_drive.isOpenLoop);
    fieldOriented.set(m_drive.m_fieldOriented);
    
    X.set(m_drive.getX());
    Y.set(m_drive.getY());
    heading.set(m_drive.getHeadingDegrees());
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    openloop.close();
    fieldOriented.close();
    X.close();
    Y.close();
    heading.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
