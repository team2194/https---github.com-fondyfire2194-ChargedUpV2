// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LiftArm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  NetworkTable liftprof = inst.getTable("liftprof");

  public DoublePublisher goalAngle;
  public DoublePublisher velocity;
  public DoublePublisher feedforward;
  public DoublePublisher pidval;
  public DoublePublisher lastspeed;;
  public DoublePublisher accel;

  private boolean directionIsDown;

  private boolean inIZone;

  public PositionProfileLift(LiftArmSubsystem lift, TrapezoidProfile.Constraints constraints, double goalAngleRadians) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lift = lift;
    m_constraints = constraints;

    m_goalAngleRadians = goalAngleRadians;

    addRequirements(m_lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    directionIsDown = m_goalAngleRadians < m_lift.getCanCoderRadians();

    loopctr = 0;

    m_lift.m_liftController.setI(0.0);

    // m_lift.m_liftController.setIntegratorRange(0, 0);

    m_lift.goalAngleRadians = m_goalAngleRadians;

    m_lift.setControllerConstraints(m_constraints);

    m_goal = new TrapezoidProfile.State(m_goalAngleRadians, 0);

    m_lift.m_liftController.reset(new TrapezoidProfile.State(m_lift.getCanCoderRadians(), 0));// start from present

    goalAngle = liftprof.getDoubleTopic("GOALANGLERADS").publish();
    velocity = liftprof.getDoubleTopic("ACTVEL").publish();
    feedforward = liftprof.getDoubleTopic("FFWD").publish();
    pidval = liftprof.getDoubleTopic("PIDVAL").publish();
    lastspeed = liftprof.getDoubleTopic("LASTSPEED").publish();
    accel = liftprof.getDoubleTopic("ACCEL").publish();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    loopctr++;

    double lastSpeed = 0;

    double lastTime = Timer.getFPGATimestamp();

    pidVal = m_lift.m_liftController.calculate(m_lift.getCanCoderRadians(), m_goalAngleRadians);

    double acceleration = (m_lift.m_liftController.getSetpoint().velocity - lastSpeed)

        / (Timer.getFPGATimestamp() - lastTime);

    m_lift.positionRadians = m_lift.getCanCoderRadians();

    m_lift.ff = m_lift.m_armFeedforward.calculate(m_lift.m_liftController.getSetpoint().position - (Math.PI / 2),
        m_lift.m_liftController.getSetpoint().velocity);

    if (directionIsDown)

      m_lift.ff *= .5;

    m_lift.m_motor.setVoltage(pidVal + m_lift.ff);

    lastSpeed = m_lift.m_liftController.getSetpoint().velocity;

    lastTime = Timer.getFPGATimestamp();

    inIZone = checkIzone(.05);

    // if (!inIZone)

    // m_lift.m_liftController.setI(0);

    // else

    // m_lift.m_liftController.setI(0.01);

    goalAngle.set(m_lift.goalAngleRadians);
    velocity.set(m_lift.getCanCoderRateRadsPerSec());
    feedforward.set(m_lift.ff);
    pidval.set(m_lift.m_liftController.getSetpoint().position);
    accel.set(m_lift.positionRadians);
    lastspeed.set(m_lift.m_liftController.getSetpoint().velocity);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    goalAngle.close();
    velocity.close();
    feedforward.close();
    pidval.close();
    lastspeed.close();
    accel.close();
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
