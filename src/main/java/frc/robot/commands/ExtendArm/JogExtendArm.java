// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ExtendArm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.subsystems.ExtendArmSubsystem;

public class JogExtendArm extends CommandBase {
  /** Creates a new JogArm. */
  private ExtendArmSubsystem m_ext;
  private CommandXboxController m_controller;
  private DoubleSupplier m_speed;
  private double throttle;

  public JogExtendArm(ExtendArmSubsystem ext, DoubleSupplier speed, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ext = ext;
    m_controller = controller;
    m_speed = speed;
    addRequirements(m_ext);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ext.resetFF = true;
    m_ext.firstUp = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double throttleMultiplier = .5;

    throttle = MathUtil.applyDeadband(Math.abs(m_speed.getAsDouble()),
        ExtendArmConstants.kControllerDeadband)
        * Math.signum(m_speed.getAsDouble());

    throttle = Math.signum(throttle) * Math.pow(throttle, 2);

    double throttle_sl = throttle;

    throttle_sl *= throttleMultiplier;

    boolean allowOut = m_ext.getPositionInches() <= ExtendArmConstants.MAX_POSITION
        || m_controller.getHID().getBackButton();

    boolean allowIn = m_ext.getPositionInches() >= ExtendArmConstants.MIN_POSITION
        || m_controller.getHID().getBackButton();

    m_ext.commandIPS = throttle_sl;

SmartDashboard.putNumber("EXTTHV", throttle_sl);
    if (throttle_sl > 0 && allowOut || throttle_sl < 0 && allowIn) {

      m_ext.m_motor.setVoltage(throttle_sl * RobotController.getBatteryVoltage());

    } else {

      m_ext.m_motor.setVoltage(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_ext.m_motor.setVoltage(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
