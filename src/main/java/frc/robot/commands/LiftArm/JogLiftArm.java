// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LiftArm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.subsystems.LiftArmSubsystem;

public class JogLiftArm extends CommandBase {
  /** Creates a new JogArm. */
  private LiftArmSubsystem m_lift;
  private CommandXboxController m_controller;
  private DoubleSupplier m_speed;
  private double throttle;
  private double throttleMultiplier = .5;

  // private final SlewRateLimiter m_slewLift = new
  // SlewRateLimiter(LiftArmConstants.JOG_SLEW_RATE, -10000, 0);

  public JogLiftArm(LiftArmSubsystem lift, DoubleSupplier speed, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lift = lift;
    m_controller = controller;
    m_speed = speed;
    addRequirements(m_lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lift.resetFF = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    throttle = MathUtil.applyDeadband(Math.abs(m_speed.getAsDouble()),
        LiftArmConstants.kControllerDeadband)
        * Math.signum(m_speed.getAsDouble());

    throttle = Math.signum(throttle) * Math.pow(throttle, 2);

    double throttle_sl = throttle;

    boolean allowUp = m_lift.getCanCoderPosition() <= LiftArmConstants.MAX_ANGLE
        || m_controller.getHID().getBackButton();

    boolean allowDown = m_lift.getCanCoderPosition() >= LiftArmConstants.MIN_ANGLE
        || m_controller.getHID().getBackButton();

    throttle_sl *= throttleMultiplier;

    double volts = throttle_sl * RobotController.getBatteryVoltage();

    //SmartDashboard.putNumber("LIFTV", volts);

    if (throttle_sl > 0 & allowUp || throttle_sl < 0 && allowDown) {

      m_lift.m_motor.setVoltage(volts);

    }

    else {

      m_lift.m_motor.setVoltage(0);
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lift.m_motor.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
