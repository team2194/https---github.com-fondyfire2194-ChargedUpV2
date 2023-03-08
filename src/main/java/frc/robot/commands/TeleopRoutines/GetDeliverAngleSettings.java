// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem.presetExtArmDistances;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem.GridDrop;
import frc.robot.subsystems.GameHandlerSubsystem.dropOffLevel;
import frc.robot.subsystems.GameHandlerSubsystem.gamePiece;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.LightStrip;
import frc.robot.subsystems.LiftArmSubsystem.presetLiftAngles;
import frc.robot.subsystems.LightStrip.ledColors;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystem.presetWristAngles;

public class GetDeliverAngleSettings extends CommandBase {
  /** Creates a new DeliverAngleSettings. */
  private LiftArmSubsystem m_lift;
  private ExtendArmSubsystem m_ext;
  private GameHandlerSubsystem m_ghs;
  private WristSubsystem m_wrist;

  private gamePiece m_piece;
  private dropOffLevel m_level;
  private GridDrop m_activeDrop;
  private boolean m_isPipe;

  private double m_liftAngle;
  private double m_extDistance;
  private double m_wristAngle;
  private boolean done;

  public GetDeliverAngleSettings(LiftArmSubsystem lift, ExtendArmSubsystem ext, WristSubsystem wrist,
      GameHandlerSubsystem ghs) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lift = lift;
    m_ext = ext;
    m_ghs = ghs;
    m_wrist = wrist;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    m_piece = m_ghs.getGamePiecetype();
    m_level = m_ghs.getDropOffLevel();
    m_activeDrop = m_ghs.getActiveDrop();
    m_isPipe = m_activeDrop.getIsPipeBlue() || m_activeDrop.getIsPipeRed();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_piece == gamePiece.CONE)
      LightStrip.setColor(ledColors.YELLOW);

    if (m_piece == gamePiece.CUBE)
      LightStrip.setColor(ledColors.PURPLE);

    if (m_level == dropOffLevel.GROUND_LEVEL) {
      done = true;
      if (m_piece == gamePiece.CUBE) {
        m_liftAngle = presetLiftAngles.PLACE_CUBE_GROUND.getAngle();
        m_extDistance = presetExtArmDistances.PLACE_CUBE_GROUND.getDistance();
        m_wristAngle = presetWristAngles.PLACE_CUBE_GROUND.getAngle();
      }
      if (m_piece == gamePiece.CONE) {
        m_liftAngle = presetLiftAngles.PLACE_CONE_GROUND.getAngle();
        m_extDistance = presetExtArmDistances.PLACE_CONE_GROUND.getDistance();
        m_wristAngle = presetWristAngles.PLACE_CONE_GROUND.getAngle();
      }
    }

    if (!done && m_level == dropOffLevel.MID_LEVEL) {
      done = true;
      if (m_piece == gamePiece.CUBE) {
        m_liftAngle = presetLiftAngles.PLACE_CUBE_MID_SHELF.getAngle();
        m_extDistance = presetExtArmDistances.PLACE_CUBE_MID_SHELF.getDistance();
        m_wristAngle = presetWristAngles.PLACE_CUBE_MID_SHELF.getAngle();
      }

      if (m_piece == gamePiece.CONE && !m_isPipe) {
        m_liftAngle = presetLiftAngles.PLACE_CONE_MID_SHELF.getAngle();
        m_extDistance = presetExtArmDistances.PLACE_CONE_MID_SHELF.getDistance();
        m_wristAngle = presetWristAngles.PLACE_CONE_MID_SHELF.getAngle();
      }
      if (m_piece == gamePiece.CONE && m_isPipe) {
        m_liftAngle = presetLiftAngles.PLACE_CONE_MID_PIPE.getAngle();
        m_extDistance = presetExtArmDistances.PLACE_CONE_MID_PIPE.getDistance();
        m_wristAngle = presetWristAngles.PLACE_CONE_MID_PIPE.getAngle();
      }

    }

    if (!done && m_level == dropOffLevel.TOP_LEVEL) {
      done = true;
      if (m_piece == gamePiece.CUBE) {
        m_liftAngle = presetLiftAngles.PLACE_CUBE_TOP_SHELF.getAngle();
        m_extDistance = presetExtArmDistances.PLACE_CUBE_TOP_SHELF.getDistance();
        m_wristAngle = presetWristAngles.PLACE_CUBE_TOP_SHELF.getAngle();
      } else {
        m_liftAngle = presetLiftAngles.PLACE_CONE_TOP_SHELF.getAngle();
        m_extDistance = presetExtArmDistances.PLACE_CONE_TOP_SHELF.getDistance();
        m_wristAngle = presetWristAngles.PLACE_CONE_TOP_SHELF.getAngle();
      }

      if (m_piece == gamePiece.CONE && !m_isPipe) {
        m_liftAngle = presetLiftAngles.PLACE_CONE_TOP_SHELF.getAngle();
        m_extDistance = presetExtArmDistances.PLACE_CONE_TOP_SHELF.getDistance();
        m_wristAngle = presetWristAngles.PLACE_CONE_TOP_SHELF.getAngle();
      }
      if (m_piece == gamePiece.CONE && m_isPipe) {
        m_liftAngle = presetLiftAngles.PLACE_CONE_TOP_PIPE.getAngle();
        m_extDistance = presetExtArmDistances.PLACE_CONE_TOP_PIPE.getDistance();
        m_wristAngle = presetWristAngles.PLACE_CONE_TOP_PIPE.getAngle();
      }
    }

    m_lift.deliverAngle = m_liftAngle;
    m_ext.deliverDistance = m_extDistance;
    m_wrist.deliverAngle = m_wristAngle;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
