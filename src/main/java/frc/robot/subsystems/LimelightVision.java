// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightVision extends SubsystemBase {
  /** Creates a new LimelightVision. */

  public Pose2d visionPoseEstimatedData;

  public double imageCaptureTime;

  public int fiducialId;

  public double llHeartbeat;

  public double llHeartbeatLast;
  // public Transform3d robotPose_FS;

  public boolean allianceBlue;

  public enum pipelinetype {
    retroreflective,
    grip,
    python,
    fiducialmarkers,
    classifier,
    detector;

    public static final pipelinetype values[] = values();
  }

  // camera ppeline names are 15
  public enum pipelines {
    REFL_LOW_TAPE(0, pipelinetype.retroreflective),
    REFL_HIGH_TAPE(1, pipelinetype.retroreflective),
    LOAD(2, pipelinetype.fiducialmarkers),
    LOAD_LEFT(3, pipelinetype.fiducialmarkers),
    LOAD_RIGHT(4, pipelinetype.fiducialmarkers),
    TAGS(5, pipelinetype.fiducialmarkers),
    DEFAULT(6, pipelinetype.fiducialmarkers),
    SPARE7(7, pipelinetype.fiducialmarkers),
    CUBE_DETECT(8, pipelinetype.detector),
    CONE_DETECT(9, pipelinetype.detector);

    public static final pipelines values[] = values();

    private int number;

    private pipelinetype type;

    public String pipelineTypeName;

    private pipelines(int number, pipelinetype type) {
      this.number = number;
      this.type = type;
    }

  }

  public int currentPipelineIndex;
  public pipelinetype currentPipelineType;
  public pipelines currentPipeline;

  private int samples;

  public boolean limelightExists;

  public String limelighttypename = "fiducial";

  public LimelightVision() {
    currentPipeline = pipelines.DEFAULT;
    // SmartDashboard.putString("LLV","LLC");
  }

  public boolean getAllianceBlue() {
    return (DriverStation.getAlliance() == Alliance.Blue);
  }

  @Override
  public void periodic() {

    if (RobotBase.isReal()) {

      llHeartbeat = LimelightHelpers.getLimelightNTDouble("limelight", "hb");
      if (llHeartbeat == llHeartbeatLast) {
        samples += 1;
      } else {
        samples = 0;
        llHeartbeatLast = llHeartbeat;
        limelightExists = true;
      }
      if (samples > 5)
        limelightExists = false;
    }
    // SmartDashboard.putBoolean("LLExists", limelightExists);

    if (RobotBase.isReal() && limelightExists) {

      fiducialId = (int) LimelightHelpers.getFiducialID("limelight");

      currentPipelineIndex = (int) LimelightHelpers.getCurrentPipelineIndex("limelight");

      currentPipeline = pipelines.values[currentPipelineIndex];

      currentPipelineType = currentPipeline.type;

      limelighttypename = getCurrentPipelineTypeName();
    }
  }

  public double round2dp(double number) {
    number = Math.round(number * 100);
    number /= 100;
    return number;
  }

  public String getCurrentPipelineName() {
    return currentPipeline.name();
  }

  public String getCurrentPipelineTypeName() {
    return currentPipeline.type.name();
  }

  public pipelinetype getCurrentPipelineType() {
    return currentPipeline.type;
  }

  public pipelines getCurrentPipeline() {
    return currentPipeline;
  }

  public void setLoadPipeline() {
    if (limelightExists)
      LimelightHelpers.setPipelineIndex("limelight", pipelines.LOAD.ordinal());
  }

  public void setLeftLoadPipeline() {
    if (limelightExists)
      LimelightHelpers.setPipelineIndex("limelight", pipelines.LOAD_LEFT.ordinal());
  }

  public void setRightLoadPipeline() {
    if (limelightExists)
      LimelightHelpers.setPipelineIndex("limelight", pipelines.LOAD_RIGHT.ordinal());
  }

  public void setHighTapePipeline() {
    if (limelightExists)
      LimelightHelpers.setPipelineIndex("limelight", pipelines.REFL_HIGH_TAPE.ordinal());
  }

  public void setLowTapePipeline() {
    if (limelightExists)
      LimelightHelpers.setPipelineIndex("limelight", pipelines.REFL_LOW_TAPE.ordinal());
  }

  public void setConeDetectorPipeline() {
    if (limelightExists)
      LimelightHelpers.setPipelineIndex("limelight", pipelines.CONE_DETECT.ordinal());
  }

  public void setCubeDetectorPipeline() {
    if (limelightExists)
      LimelightHelpers.setPipelineIndex("limelight", pipelines.CUBE_DETECT.ordinal());
  }

}
