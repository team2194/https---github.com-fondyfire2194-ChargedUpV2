// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.TrajectoryFactory;

public class EnsureDriveStartFromVision extends CommandBase {
  /** Creates a new TrajToLoadStart. */
  private DriveSubsystem m_drive;
  private TrajectoryFactory m_tf;
  private Pose2d[] samplePose = new Pose2d[4];
  private int samplePointer;
  private boolean samplesTaken;
  double[] result0 = new double[3];
  double[] result1 = new double[3];
  double[] result2 = new double[3];

  private boolean complete;

  public EnsureDriveStartFromVision(DriveSubsystem drive, TrajectoryFactory tf) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_tf = tf;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    samplePointer = 0;
    complete = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotBase.isReal()) {

      samplesTaken = samplePointer >= samplePose.length - 1;

      if (!samplesTaken) {

        // SmartDashboard.putNumber("SAMPLEPTR", samplePointer);

        // SmartDashboard.putBoolean("SAMPLESTAKEN", samplesTaken);

        samplePose[samplePointer] = m_drive.botPose;

        samplePointer++;
      }
      if (samplesTaken) {
        
        result0 = m_drive.r2dToArray(samplePose[0]);
        result1 = m_drive.r2dToArray(samplePose[1]);
        result2 = m_drive.r2dToArray(samplePose[2]);
        samplePointer = 0;

        if (m_drive.posesCompare(result0, result1, result2)) {
          m_tf.startLoadPose = samplePose[1];
          complete = true;
          // SmartDashboard.putString("filterpose", samplePose[1].toString());

        }
      }
    } else {

      m_tf.startLoadPose = m_drive.getEstimatedPosition();

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return complete;
  }
}
