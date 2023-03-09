// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

    m_wrist.m_wristController.setIntegratorRange(-2, 2);

    if (setController) {

      m_goal = new TrapezoidProfile.State(m_goalAngleRadians, 0);

      m_wrist.m_wristController.reset(new TrapezoidProfile.State(m_wrist.getAngleRadians(), 0));
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    loopctr++;

    double lastSpeed = 0;

    double lastTime = Timer.getFPGATimestamp();

    m_wrist.pidVal = m_wrist.m_wristController.calculate(m_wrist.getAngleRadians(), m_wrist.goalAngleRadians);

    double acceleration = (m_wrist.m_wristController.getSetpoint().velocity - lastSpeed)

        / (Timer.getFPGATimestamp() - lastTime);

    double cancodeAngle = m_lift.getCanCoderRadians();

    cancodeAngle = 78;

  
    m_wrist.ff = m_wrist.m_armfeedforward.calculate(
        m_wrist.m_wristController.getSetpoint().position - cancodeAngle - (Math.PI / 2),
        m_wrist.m_wristController.getSetpoint().velocity);

    m_wrist.gravCalc = m_wrist.m_wristController.getSetpoint().position - cancodeAngle - (Math.PI / 2);

    double pidvolts = m_wrist.pidVal * RobotController.getBatteryVoltage();

    m_wrist.m_motor.setVoltage(pidvolts + m_wrist.ff);

    lastSpeed = m_wrist.m_wristController.getSetpoint().velocity;

    lastTime = Timer.getFPGATimestamp();

    inIZone = checkIzone(.1);

    if (!inIZone && m_wrist.m_wristController.getI() != 0)

      m_wrist.m_wristController.setI(0);

    if (inIZone && m_wrist.m_wristController.getI() == 0)

      m_wrist.m_wristController.setI(0.5);
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
