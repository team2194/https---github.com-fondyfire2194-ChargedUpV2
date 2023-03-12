// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LiftArm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.subsystems.LiftArmSubsystem;

public class PositionProfileLift extends CommandBase {
  /** Creates a new PositionArm. */
  private LiftArmSubsystem m_lift;

  private TrapezoidProfile.Constraints m_constraints;

  private TrapezoidProfile.State m_goal;

  // private TrapezoidProfile.State m_setpoint;

  private double m_goalAngleRadians;

  private double pidVal;

  private int loopctr;

  private boolean directionIsDown;

  private boolean setController;

  public PositionProfileLift(LiftArmSubsystem lift, TrapezoidProfile.Constraints constraints, double goalAngleRadians) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lift = lift;
    m_constraints = constraints;

    m_goalAngleRadians = goalAngleRadians;

    addRequirements(m_lift);
  }

  public PositionProfileLift(LiftArmSubsystem lift) {
    m_lift = lift;
    addRequirements(m_lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    directionIsDown = m_goalAngleRadians < m_lift.getCanCoderRadians();

    loopctr = 0;

    m_lift.m_liftController.setI(0.0);

    m_lift.m_liftController.setTolerance(Units.degreesToRadians(1));

    if (setController) {

      m_lift.goalAngleRadians = m_goalAngleRadians;

      m_lift.setController(m_constraints, m_goalAngleRadians, false);

      m_goal = new TrapezoidProfile.State(m_goalAngleRadians, 0);

      m_lift.m_liftController.reset(new TrapezoidProfile.State(m_lift.getCanCoderRadians(), 0));// start from present

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean allowUp = m_lift.getCanCoderPosition() <= LiftArmConstants.MAX_ANGLE;

    boolean allowDown = m_lift.getCanCoderPosition() >= LiftArmConstants.MIN_ANGLE;

    loopctr++;


    pidVal = m_lift.m_liftController.calculate(m_lift.getCanCoderRadians(), m_lift.goalAngleRadians);

    m_lift.ff = m_lift.m_armFeedforward.calculate(m_lift.m_liftController.getSetpoint().position - (Math.PI / 2),

        m_lift.m_liftController.getSetpoint().velocity);

    m_lift.volts = pidVal + m_lift.ff;

    if (allowUp && m_lift.volts > 0 || allowDown && m_lift.volts < 0) {

      m_lift.m_motor.setVoltage(m_lift.volts);

      m_lift.inIZone = checkIzone(.1);

      if ((!m_lift.inIZone || m_lift.m_liftController.atSetpoint()) && m_lift.m_liftController.getI() != 0) {

        m_lift.m_liftController.setI(0);

        m_lift.m_liftController.setIntegratorRange(0, 0);

      }

      if (m_lift.inIZone && m_lift.m_liftController.getI() == 0) {

        m_lift.m_liftController.setI(0.001);

        m_lift.m_liftController.setIntegratorRange(-.02, .02);
      }

    } else

      m_lift.m_motor.setVoltage(0);

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
    return Math.abs(m_lift.goalAngleRadians - m_lift.getCanCoderRadians()) < izonelimit;

  }

}
