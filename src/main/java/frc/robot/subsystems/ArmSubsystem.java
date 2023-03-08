// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.WristConstants;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends ProfiledPIDSubsystem {
  public final CANSparkMax m_motor = new CANSparkMax(CanConstants.WRIST_MOTOR,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  public GenericEntry wristTarget;
  private final ArmFeedforward m_feedforward =

      new ArmFeedforward(
          WristConstants.ksVolts, WristConstants.kgVolts,
          WristConstants.kVVoltSecondPerRad, WristConstants.kAVoltSecondSquaredPerRad);

  /** Create a new ArmSubsystem. */
  public ArmSubsystem() {
    super(
        new ProfiledPIDController(
            .06,
            0,
            0,
            new TrapezoidProfile.Constraints(
                WristConstants.kMaxVelocityRadPerSecond,
                WristConstants.kMaxAccelerationRadPerSecSquared)),
        0);
    m_encoder.setPositionConversionFactor(WristConstants.RADIANS_PER_ENCODER_REV);
    m_motor.restoreFactoryDefaults();
    m_motor.setInverted(true);
    m_motor.setOpenLoopRampRate(1);
    m_motor.setClosedLoopRampRate(1);

    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);// vel

    m_encoder.setPositionConversionFactor(WristConstants.RADIANS_PER_ENCODER_REV);

    m_encoder.setVelocityConversionFactor(WristConstants.RADIANS_PER_ENCODER_REV / 60);

    SmartDashboard.putNumber("WRDPR", WristConstants.RADIANS_PER_ENCODER_REV);

    // setGoal(9);

    wristTarget = Shuffleboard.getTab("Arms")
        .add("TargetAngleDeg", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withPosition(4, 3).withSize(2, 1)
        .withProperties(Map.of("min", 0, "max", 90))
        .getEntry();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_motor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    return m_encoder.getPosition();
  }

  public boolean checkCANOK() {
    return RobotBase.isSimulation() || m_motor.getFirmwareVersion() != 0;
  }

}
