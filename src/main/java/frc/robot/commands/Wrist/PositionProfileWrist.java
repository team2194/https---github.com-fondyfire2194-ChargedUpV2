// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class PositionProfileWrist extends CommandBase {
  /** Creates a new PositionArm. */
  private WristSubsystem m_wrist;

  private LiftArmSubsystem m_lift;

  private TrapezoidProfile.Constraints m_constraints;

  private TrapezoidProfile.State m_goal;

  private double m_goalAngleRadians;

  private int loopctr;

  private boolean inIZone;
  
  private boolean setController;

  public PositionProfileWrist(WristSubsystem wrist, LiftArmSubsystem lift, TrapezoidProfile.Constraints constraints,
      double goalAngleRadians) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_wrist = wrist;
    m_lift = lift;
    m_constraints = constraints;
    m_goalAngleRadians = goalAngleRadians;
    setController = true;

    addRequirements(m_wrist);
  }

  public PositionProfileWrist(WristSubsystem wrist, LiftArmSubsystem lift) {
    m_wrist = wrist;
    m_lift = lift;
    setController = false;
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    loopctr = 0;

    m_wrist.m_wristController.setI(0);

    m_wrist.m_wristController.setTolerance(Units.degreesToRadians(1));

    if (setController) {

      m_goal = new TrapezoidProfile.State(m_goalAngleRadians, 0);

      m_wrist.m_wristController.reset(new TrapezoidProfile.State(m_wrist.getAngleRadians(), 0));
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean allowUp = m_wrist.getAngleDegrees() <= WristConstants.MAX_ANGLE;

    boolean allowDown = m_wrist.getAngleDegrees() >= WristConstants.MIN_ANGLE;

    loopctr++;

    m_wrist.pidVal = m_wrist.m_wristController.calculate(m_wrist.getAngleRadians(), m_wrist.goalAngleRadians);

    m_wrist.ff = m_wrist.m_armfeedforward.calculate(
        m_wrist.m_wristController.getSetpoint().position - m_wrist.getAngleRadians() - (Math.PI / 2)
            - m_lift.getCanCoderRadians(),
        m_wrist.m_wristController.getSetpoint().velocity);

    m_wrist.volts = m_wrist.pidVal + m_wrist.ff;

    if (allowDown && allowUp) {

      m_wrist.m_motor.setVoltage(m_wrist.volts);

      inIZone = checkIzone(.1);

      if ((m_wrist.m_wristController.atSetpoint() || !inIZone) && m_wrist.m_wristController.getI() != 0){

        m_wrist.m_wristController.setI(0);


      }

      if (inIZone && m_wrist.m_wristController.getI() == 0){

        m_wrist.m_wristController.setI(0.001);


      }
    }

    else {

      m_wrist.m_motor.setVoltage(0);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean checkIzone(double izonelimit) {
    return Math.abs(m_wrist.goalAngleRadians - m_wrist.getAngleRadians()) < izonelimit;

  }

}
