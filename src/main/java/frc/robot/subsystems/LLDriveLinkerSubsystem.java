// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.simulation.SimConstants;
import frc.robot.subsystems.LimelightVision.pipelines;
import frc.robot.subsystems.LimelightVision.pipelinetype;

public class LLDriveLinkerSubsystem extends SubsystemBase {
  /**
   * Creates a new LLD
   */
  private LimelightVision m_llv;
  private DriveSubsystem m_drive;
  private LimelightResults llresults;
  private double llHeartbeat;
  private double llHeartbeatLast;
  private int samples;
  private boolean limelightExists;
  private int loopctr;
  private double neuralClassID;
  private double coneID = 0;
  private double cubeID = 1;

  public LLDriveLinkerSubsystem(LimelightVision llv, DriveSubsystem drive) {
    m_llv = llv;
    m_drive = drive;
  }

  @Override
  public void periodic() {
    loopctr++;

    llHeartbeat = LimelightHelpers.getLimelightNTDouble("limelight", "hb");

    if (llHeartbeat == llHeartbeatLast) {
      samples += 1;
    } else {
      samples = 0;
      llHeartbeatLast = llHeartbeat;
      limelightExists = true;
      m_drive.limelightExists = true;
      m_llv.limelightExists = true;
    }

    if (samples > 5) {
      limelightExists = false;
      m_drive.limelightExists = false;
      m_llv.limelightExists = false;
    }

    if (RobotBase.isReal() && limelightExists && loopctr > 2 && !m_drive.inhibitVision) {

      loopctr = 0;

      llresults = LimelightHelpers.getLatestResults("limelight");

      if (m_llv.getCurrentPipelineType() == pipelinetype.fiducialmarkers) {
        
        m_drive.botPose = getBotPose();
        m_drive.numberTags = llresults.targetingResults.targets_Fiducials.length;
        m_drive.hasTag = m_drive.numberTags > 0;
        m_drive.fiducialID = (int) LimelightHelpers.getFiducialID("limelight");

        m_drive.latencyCaptureMs = LimelightHelpers.getLatency_Capture("limelight");
        m_drive.latencyCaptureMs /= 1000;
        m_drive.latencyPipelineMs = LimelightHelpers.getLatency_Pipeline("limelight");
        m_drive.latencyPipelineMs /= 1000;

        if (m_drive.numberTags >= 1)
          m_drive.tagid1 = (int) llresults.targetingResults.targets_Fiducials[0].fiducialID;

        m_drive.tagid2 = -1;
        if (m_drive.numberTags >= 2)
          m_drive.tagid2 = (int) llresults.targetingResults.targets_Fiducials[1].fiducialID;

      }

      if (m_llv.getCurrentPipelineType() == pipelinetype.retroreflective) {

        m_drive.numberTargets = llresults.targetingResults.targets_Retro.length;
        m_drive.hasTarget = m_drive.numberTargets > 0;
        m_drive.tx = LimelightHelpers.getTX("limelight");
        m_drive.ty = LimelightHelpers.getTY("limelight");
        m_drive.targetArea = LimelightHelpers.getTA("limelight");

      }

      if (m_llv.getCurrentPipeline() == pipelines.CONE_DETECT) {

        neuralClassID = LimelightHelpers.getNeuralClassID("limelight");

        m_drive.coneFound = neuralClassID == coneID;

        // String temp = LimelightHelpers.getNeuralClassName("limelight");
        // SmartDashboard.putString("NCN1", temp);
        m_drive.tx = LimelightHelpers.getTX("limelight");
        m_drive.ty = LimelightHelpers.getTY("limelight");
        m_drive.targetArea = LimelightHelpers.getTA("limelight");
      }

      else {
        m_drive.coneFound = false;
      }

      if (m_llv.getCurrentPipeline() == pipelines.CUBE_DETECT) {

        neuralClassID = LimelightHelpers.getNeuralClassID("limelight");

        m_drive.cubeFound = neuralClassID == cubeID;

        m_drive.tx = LimelightHelpers.getTX("limelight");
        m_drive.ty = LimelightHelpers.getTY("limelight");
        m_drive.targetArea = LimelightHelpers.getTA("limelight");
      }

      else {

        m_drive.cubeFound = false;

      }
    }
  }

  public Pose2d getBotPose() {
    Pose2d temp = new Pose2d();
    if (DriverStation.getAlliance() == Alliance.Blue)
      temp = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
    else
      temp = LimelightHelpers.getBotPose2d_wpiRed("limelight");
    if (poseIsOnField(temp) && poseChangeInRange(temp))
      return temp;
    else
      return temp;
  }

  public boolean poseIsOnField(Pose2d pose) {
    return pose.getX() <= SimConstants.fieldLength &&
        pose.getY() <= SimConstants.fieldWidth
        && pose.getRotation().getDegrees() <= 180
        && pose.getRotation().getDegrees() >= -180;
  }

  public boolean poseChangeInRange(Pose2d pose) {
    return true;
    // return Math.abs(pose.getX() - lastPose.getX()) < maxXChange
    // && Math.abs(pose.getY() - lastPose.getY()) < maxYChange;
  }

}
