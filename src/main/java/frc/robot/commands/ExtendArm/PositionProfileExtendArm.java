// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ExtendArm;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private boolean inIZone;

  private boolean setController;

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

    loopctr = 0;

    m_ext.m_extController.setI(0);

    m_ext.m_extController.setIntegratorRange(-2, 2);

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

    loopctr++;

    double lastSpeed = 0;

    double lastTime = Timer.getFPGATimestamp();

    m_ext.pidVal = m_ext.m_extController.calculate(m_ext.getPositionInches(),
        m_ext.goalInches);

    double acceleration = (m_ext.m_extController.getSetpoint().velocity -
        lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

    m_ext.ff = m_ext.m_feedforward.calculate(m_ext.m_extController.getSetpoint().velocity,
        acceleration);

    double profvel = m_ext.m_extController.getSetpoint().velocity;

    double pidvolts = m_ext.pidVal * RobotController.getBatteryVoltage();

    if (allowIn && m_ext.ff < 0 || allowOut && m_ext.ff > 0) {

      m_ext.m_motor.setVoltage(pidvolts + m_ext.ff);

    } else {

      m_ext.m_motor.setVoltage(0);
    }

    lastSpeed = m_ext.m_extController.getSetpoint().velocity;

    lastTime = Timer.getFPGATimestamp();

    inIZone = checkIzone(2.0);

    if (!inIZone && m_ext.m_extController.getI() != 0)

      m_ext.m_extController.setI(0);

    if (inIZone && m_ext.m_extController.getI() == 0)

      m_ext.m_extController.setI(0.5);
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
