// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LiftArm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
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

  private boolean inIZone;

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

    // m_lift.m_liftController.setIntegratorRange(0, 0);

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

    loopctr++;

    double lastSpeed = 0;

    double lastTime = Timer.getFPGATimestamp();

    pidVal = m_lift.m_liftController.calculate(m_lift.getCanCoderRadians(), m_lift.goalAngleRadians);

    double acceleration = (m_lift.m_liftController.getSetpoint().velocity - lastSpeed)

        / (Timer.getFPGATimestamp() - lastTime);

    m_lift.positionRadians = m_lift.getCanCoderRadians();

    m_lift.ff = m_lift.m_armFeedforward.calculate(m_lift.m_liftController.getSetpoint().position - (Math.PI / 2),

        m_lift.m_liftController.getSetpoint().velocity);

    m_lift.gravCalc = m_lift.m_liftController.getSetpoint().position - m_lift.positionRadians - (Math.PI / 2);

    m_lift.m_motor.setVoltage((pidVal * RobotController.getBatteryVoltage()) + m_lift.ff);

    lastSpeed = m_lift.m_liftController.getSetpoint().velocity;

    lastTime = Timer.getFPGATimestamp();

    inIZone = checkIzone(.1);

    if (!inIZone)

      m_lift.m_liftController.setI(0);

    else

      m_lift.m_liftController.setI(0.1);

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
