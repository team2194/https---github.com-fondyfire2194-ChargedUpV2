// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private boolean setController;

  private double lastSpeed = 0;

  private double lastTime;

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

    lastTime = Timer.getFPGATimestamp();

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


    m_wrist.pidVal = m_wrist.m_wristController.calculate(m_wrist.getAngleRadians(),
        m_wrist.goalAngleRadians);

    // double temp = m_wrist.pidVal * RobotController.getBatteryVoltage();

    // m_wrist.pidVal = MathUtil.clamp(temp, -1, 1);

    double acceleration = (m_wrist.m_wristController.getSetpoint().velocity -

        lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

    m_wrist.ff = m_wrist.m_armfeedforward.calculate(
        m_wrist.m_wristController.getSetpoint().position - m_wrist.getAngleRadians() - (Math.PI / 2)
            - m_lift.getCanCoderRadians(),
        m_wrist.m_wristController.getSetpoint().velocity, acceleration);

    m_wrist.volts = m_wrist.pidVal + m_wrist.ff;

    if (allowDown && m_wrist.volts < 0 || allowUp && m_wrist.volts > 0) {

      m_wrist.m_motor.setVoltage(m_wrist.volts);

      lastSpeed = m_wrist.m_wristController.getSetpoint().velocity;

      lastTime = Timer.getFPGATimestamp();

      m_wrist.inIZone = checkIzone(.1);

      if ((m_wrist.m_wristController.atGoal() || !m_wrist.inIZone) && m_wrist.m_wristController.getI() != 0) {

        m_wrist.m_wristController.setI(0);

      }

      if (m_wrist.inIZone && m_wrist.m_wristController.getI() == 0) {

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
