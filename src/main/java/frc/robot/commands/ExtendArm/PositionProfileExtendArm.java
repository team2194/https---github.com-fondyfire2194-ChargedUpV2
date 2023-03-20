// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ExtendArm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
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

  private boolean setController;

  private double lastSpeed = 0;

  private double lastTime;

  public PositionProfileExtendArm(ExtendArmSubsystem ext, LiftArmSubsystem lift,
      TrapezoidProfile.Constraints constraints, double goalInches) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ext = ext;

    m_lift = lift;

    m_constraints = constraints;

    m_goalInches = goalInches;

    setController = true;

    addRequirements(m_ext);
  }

  public PositionProfileExtendArm(ExtendArmSubsystem ext, LiftArmSubsystem lift) {
    m_ext = ext;
    m_lift = lift;
    addRequirements(m_ext);
    setController = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    lastTime = Timer.getFPGATimestamp();

    loopctr = 0;

    m_ext.m_extController.setI(0);

    m_ext.m_extController.setTolerance(1);

    if (setController) {

      m_ext.setControllerConstraints(m_constraints);

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

    boolean directionIsOut = m_goalInches > m_ext.getPositionInches();
    loopctr++;

    m_ext.gravVal = -Pref.getPref("extKg") * Math.cos(m_lift.getCanCoderRadians());

    // if (directionIsOut)
    //   m_ext.gravVal *= -1;

    m_ext.pidVal = m_ext.m_extController.calculate(m_ext.getPositionInches(),
        m_ext.goalInches);

    // double temp = m_ext.pidVal * RobotController.getBatteryVoltage();

    // m_ext.pidVal = MathUtil.clamp(temp, -1, 1);

    double acceleration = (m_ext.m_extController.getSetpoint().velocity -
        lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

    m_ext.ff = m_ext.m_feedforward.calculate(m_ext.m_extController.getSetpoint().velocity,
        acceleration);

    m_ext.volts = m_ext.ff + m_ext.pidVal - m_ext.gravVal;

    if (allowIn && m_ext.volts < 0 || allowOut && m_ext.volts > 0) {

      m_ext.m_motor.setVoltage(m_ext.volts);

    } else

    {

      m_ext.m_motor.setVoltage(0);
    }

    lastSpeed = m_ext.m_extController.getSetpoint().velocity;

    lastTime = Timer.getFPGATimestamp();

    // m_ext.inIZone = checkIzone(2.0);

    // if ((m_ext.m_extController.atGoal() || !m_ext.inIZone) &&
    // m_ext.m_extController.getI() != 0) {

    // m_ext.m_extController.setI(0);

    // m_ext.m_extController.setIntegratorRange(0, 0);

    // }

    // if (m_ext.inIZone && m_ext.m_extController.getI() == 0) {

    // m_ext.m_extController.setI(0.001);

    // m_ext.m_extController.setIntegratorRange(-.02, .02);

    // }
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

    return Math.abs(m_ext.goalInches - m_ext.getPositionInches()) < izonelimit;

  }

}
