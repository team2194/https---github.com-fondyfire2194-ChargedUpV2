// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ExtendArm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;

public class PositionProfileExtendArm extends CommandBase {
  /** Creates a new PositionArm. */
  private ExtendArmSubsystem m_ext;

  private LiftArmSubsystem m_lift;

  private TrapezoidProfile.Constraints m_constraints;

  private TrapezoidProfile.State m_goal;

  private double m_goalInches;

  private int loopctr;

  private double error;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  NetworkTable extprof = inst.getTable("extprof");

  public DoublePublisher goalinches;
  public DoublePublisher velocity;
  public DoublePublisher distance;
  public DoublePublisher feedforward;
  public DoublePublisher pidval;
  public DoublePublisher lastspeed;;
  public DoublePublisher accel;
  public DoublePublisher profpos;
  public DoublePublisher disterr;

  private boolean inIZone;

  private boolean setController;

  public PositionProfileExtendArm(ExtendArmSubsystem ext,LiftArmSubsystem lift, TrapezoidProfile.Constraints constraints, double goalInches) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ext = ext;

    m_lift=lift;

    m_constraints = constraints;

    m_goalInches = goalInches;

    setController = true;

    addRequirements(m_ext);
  }

  public PositionProfileExtendArm(ExtendArmSubsystem ext,LiftArmSubsystem lift) {
    m_ext = ext;
    m_lift=lift;
    addRequirements(m_ext);
    setController = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    goalinches = extprof.getDoubleTopic("GOALINCH").publish();
    velocity = extprof.getDoubleTopic("ACTVEL").publish();
    distance = extprof.getDoubleTopic("ACTDIST").publish();
    feedforward = extprof.getDoubleTopic("FFWD").publish();
    pidval = extprof.getDoubleTopic("PIDVAL").publish();
    lastspeed = extprof.getDoubleTopic("LASTSPEED").publish();
    accel = extprof.getDoubleTopic("ACCEL").publish();
    profpos = extprof.getDoubleTopic("PROFILEPOSN").publish();
    disterr = extprof.getDoubleTopic("DISTERR").publish();
  
    

    loopctr = 0;

    if (setController) {

      m_ext.setControllerConstraints(m_constraints);

      m_ext.m_extController.setI(0);

      // m_ext.m_extController.setIntegratorRange(0, 0);

      m_ext.goalInches = m_goalInches;

      m_goal = new TrapezoidProfile.State(m_ext.goalInches, 0);

      m_ext.m_extController.reset(new TrapezoidProfile.State(m_ext.getPositionInches(), 0));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean allowOut = m_ext.getPositionInches() <= ExtendArmConstants.MAX_POSITION;

    boolean allowIn = m_ext.getPositionInches() >= ExtendArmConstants.MIN_POSITION;

    loopctr++;

    double lastSpeed = 0;

    double lastTime = Timer.getFPGATimestamp();

    double pidVal = m_ext.m_extController.calculate(m_ext.getPositionInches(),
        m_ext.goalInches);

    double acceleration = (m_ext.m_extController.getSetpoint().velocity -
        lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

    m_ext.ff = m_ext.m_feedforward.calculate(m_ext.m_extController.getSetpoint().velocity,
        acceleration);

    if (allowIn && allowOut)

      m_ext.m_motor.setVoltage(pidVal + m_ext.ff);

    else

      m_ext.m_motor.setVoltage(0);

    lastSpeed = m_ext.m_extController.getSetpoint().velocity;

    lastTime = Timer.getFPGATimestamp();

    // inIZone = checkIzone(.05);

    // if (!inIZone)

    // m_ext.m_extController.setI(0);

    // else

    // m_ext.m_extController.setI(0.01);

    goalinches.set(m_goalInches);
    velocity.set(m_ext.getInchesPerSec());
    distance.set(m_ext.getPositionInches());
    feedforward.set(m_ext.ff);
    pidval.set(pidVal);
    accel.set(acceleration);
    lastspeed.set(lastSpeed);
    profpos.set(m_ext.m_extController.getSetpoint().position);
    disterr.set(m_goalInches - m_ext.getPositionInches());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ext.endComm = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ext.endComm;
  }

  private boolean checkIzone(double izonelimit) {
    return Math.abs(m_ext.goalInches - m_ext.getPositionInches()) < izonelimit;

  }

}
