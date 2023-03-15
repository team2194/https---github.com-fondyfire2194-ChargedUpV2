// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LiftArm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.subsystems.LiftArmSubsystem;

public class PositionProfileLiftInches extends CommandBase {
  /** Creates a new PositionArm. */
  private LiftArmSubsystem m_lift;

  private TrapezoidProfile.Constraints m_constraints;

  private TrapezoidProfile.State m_goal;

  // private TrapezoidProfile.State m_setpoint;

  private double m_goalInches;

  private int loopctr;

  private boolean directionIsDown;

  private boolean setController;

  private double lastSpeed = 0;

  private double lastTime;

  public PositionProfileLiftInches(LiftArmSubsystem lift, TrapezoidProfile.Constraints constraints,
      double goalInches) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lift = lift;
    m_constraints = constraints;

    m_goalInches = goalInches;

    addRequirements(m_lift);
  }

  public PositionProfileLiftInches(LiftArmSubsystem lift) {
    m_lift = lift;
    addRequirements(m_lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // directionIsDown = m_goalIn < m_lift.getCanCoderRadians();

    loopctr = 0;

    m_lift.m_liftController.setI(0.0);

    m_lift.m_liftController.setTolerance(.5);

    lastTime = Timer.getFPGATimestamp();

    if (setController) {

      m_lift.goalInches = m_goalInches;

      m_lift.setController(m_constraints, m_goalInches, false);

      m_goal = new TrapezoidProfile.State(m_goalInches, 0);

      m_lift.m_liftController.reset(new TrapezoidProfile.State(m_lift.getPositionInches(), 0));// start from present

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean allowUp = m_lift.getCanCoderPosition() <= LiftArmConstants.MAX_ANGLE;

    boolean allowDown = m_lift.getCanCoderPosition() >= LiftArmConstants.MIN_ANGLE;

    loopctr++;

    m_lift.pidVal = m_lift.m_liftController.calculate(m_lift.getPositionInches(), m_lift.goalInches);

    // double temp = m_lift.pidVal * RobotController.getBatteryVoltage();

    // m_lift.pidVal = MathUtil.clamp(temp, -1, 1);

    double acceleration = (m_lift.m_liftController.getSetpoint().velocity -

        lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

    m_lift.ff = m_lift.m_armFeedforward.calculate(m_lift.m_liftController.getSetpoint().position - (Math.PI / 2),

        m_lift.m_liftController.getSetpoint().velocity, acceleration);

    m_lift.volts = m_lift.pidVal + m_lift.ff;

    if (allowUp && m_lift.volts > 0 || allowDown && m_lift.volts < 0) {

      m_lift.m_motor.setVoltage(m_lift.volts);

      lastSpeed = m_lift.m_liftController.getSetpoint().velocity;

      lastTime = Timer.getFPGATimestamp();

      m_lift.inIZone = checkIzone(.2);

      if ((!m_lift.inIZone || m_lift.m_liftController.atGoal()) && m_lift.m_liftController.getI() != 0) {

        m_lift.m_liftController.setI(0);

        m_lift.m_liftController.setIntegratorRange(0, 0);

      }

      if (m_lift.inIZone && m_lift.m_liftController.getI() == 0) {

        m_lift.m_liftController.setI(0.001);

        m_lift.m_liftController.setIntegratorRange(-.02, .02);
      }

    } else {

      m_lift.m_motor.setVoltage(0);

      lastSpeed = 0;

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
    return Math.abs(m_lift.goalAngleRadians - m_lift.getCanCoderRadians()) < izonelimit;

  }

}
