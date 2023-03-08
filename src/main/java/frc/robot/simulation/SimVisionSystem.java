// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SimVisionSystem {
  /** Creates a new SimVision. */// Simulated Vision System.
  // Configure these to match your PhotonVision Camera,
  // pipeline, and LED setup.
  double camDiagFOV = 70.0; // degrees - assume wide-angle camera
  double camPitch = Units.radiansToDegrees(0); // degrees
  double camHeightOffGround = 1; // meters
  double maxLEDRange = 20; // meters
  int camResolutionWidth = 640; // pixels
  int camResolutionHeight = 480; // pixels
  double minTargetArea = 10; // square pixels

  public SimVisionSystem() {
    double targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters
    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 197
    double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters
    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    // pages 4 and 5
    double tgtXPos = Units.feetToMeters(54);
    double tgtYPos = Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
    Pose2d farTargetPose = new Pose2d(new Translation2d(tgtXPos, tgtYPos), new Rotation2d(0.0));
    // Field2d field = new Field2d();

    // simVision.addSimVisionTarget(
    // new SimVisionTarget(farTargetPose, VisionConstants.TARGET_HEIGHT_METERS,
    // targetWidth, targetHeight));
    // SmartDashboard.putData("Field", field);
  }

  public void periodic() {
    // This method will be called once per scheduler run
    // simVision.processFrame(drivetrainSimulator.getPose());
  }
}
