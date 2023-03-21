// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeliverRoutines;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem.presetExtArmDistances;
import frc.robot.subsystems.GameHandlerSubsystem.robotPiece;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.LiftArmSubsystem.presetLiftAngles;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystem.presetWristAngles;

public class GetDeliverAngleSettings extends CommandBase {
  /** Creates a new DeliverAngleSettings. */
  private LiftArmSubsystem m_lift;
  private ExtendArmSubsystem m_ext;
  private WristSubsystem m_wrist;
  private IntakeSubsystem m_intake;

  private robotPiece m_piece;

  private boolean m_topLevel;

  private double m_liftInches;
  private double m_liftDegrees;

  private double m_extDistance;
  private double m_wristAngleRads;
  private double m_intakeSpeed;
  private boolean done;

  public GetDeliverAngleSettings(LiftArmSubsystem lift, ExtendArmSubsystem ext, WristSubsystem wrist,
      IntakeSubsystem intake, boolean topLevel) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lift = lift;
    m_ext = ext;
    m_wrist = wrist;
    m_intake = intake;
    m_topLevel = topLevel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    m_piece = m_intake.piece;
    m_liftInches = 0;
    m_liftDegrees = presetLiftAngles.SAFE_HOME.getAngle();
    m_extDistance = 0;
    m_wristAngleRads = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (m_piece == robotPiece.CONE)
    // LightStrip.setColor(ledColors.YELLOW);

    // if (m_piece == robotPiece.CUBE)
    // LightStrip.setColor(ledColors.PURPLE);

    m_lift.topLevel = m_topLevel;

    if (!done && !m_topLevel) {
      done = true;
      if (m_intake.cubePresent) {
        m_liftInches = presetLiftAngles.PLACE_CUBE_MID_SHELF.getInches();
        m_liftDegrees = presetLiftAngles.PLACE_CUBE_MID_SHELF.getAngle();
        m_extDistance = presetExtArmDistances.PLACE_CUBE_MID_SHELF.getDistance();
        m_wristAngleRads = presetWristAngles.PLACE_CUBE_MID_SHELF.getAngleRads();
        m_intakeSpeed = Pref.getPref("cubedelspeed");// presetIntakeSpeeds.DELIVER_CUBE.getSpeed();
      }

      if (m_intake.conePresent) {
        m_liftInches = presetLiftAngles.PLACE_CONE_MID_PIPE.getInches();
        m_liftDegrees = presetLiftAngles.PLACE_CONE_MID_PIPE.getAngle();
        m_extDistance = presetExtArmDistances.PLACE_CONE_MID_PIPE.getDistance();
        m_wristAngleRads = presetWristAngles.PLACE_CONE_MID_PIPE.getAngleRads();
        m_intakeSpeed = Pref.getPref("conedelspeed");// presetIntakeSpeeds.DELIVER_CONE.getSpeed();
      }

    }

    if (!done && m_topLevel) {
      done = true;
      if (m_intake.cubePresent) {
        m_liftInches = presetLiftAngles.PLACE_CUBE_TOP_SHELF.getInches();
        m_liftDegrees = presetLiftAngles.PLACE_CUBE_TOP_SHELF.getAngle();
        m_extDistance = presetExtArmDistances.PLACE_CUBE_TOP_SHELF.getDistance();
        m_wristAngleRads = presetWristAngles.PLACE_CUBE_TOP_SHELF.getAngleRads();
        m_intakeSpeed = Pref.getPref("cubedelspeed"); // presetIntakeSpeeds.DELIVER_CUBE.getSpeed();
      }

      if (m_intake.conePresent) {
        m_liftInches = presetLiftAngles.PLACE_CONE_TOP_PIPE.getInches();
        m_liftDegrees = presetLiftAngles.PLACE_CONE_TOP_PIPE.getAngle();
        m_extDistance = presetExtArmDistances.PLACE_CONE_TOP_PIPE.getDistance();
        m_wristAngleRads = presetWristAngles.PLACE_CONE_TOP_PIPE.getAngleRads();
        m_intakeSpeed = Pref.getPref("conedelspeed");// presetIntakeSpeeds.DELIVER_CONE.getSpeed();
      }

    }

    m_lift.deliverInches = m_liftInches;
    m_lift.deliverAngle = m_liftDegrees;
    m_ext.deliverDistance = m_extDistance;
    m_wrist.deliverAngleRads = m_wristAngleRads;
    m_intake.deliverSpeed = m_intakeSpeed;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new DeliverPiecePositions(m_lift, m_ext, m_wrist, m_intake).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
