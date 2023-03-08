// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StrafeToBlueFieldY extends PIDCommand {
  /** Creates a new RotateToAngle. */

  public StrafeToBlueFieldY(DriveSubsystem drive, GameHandlerSubsystem ghs) {
    super(
        // The controller that the command will use
        drive.strafePID,
        // This should return the measurement
        () -> drive.getY(),
        // This should return the setpoint (can also be a constant)
        () -> ghs.getActiveDropPose().getY(),
        // This uses the output
        output -> {
          drive.drive(0, -output, 0);
        }, drive);
    super.m_controller.setTolerance(.1);

    // this number could be changed

    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.m_controller.atSetpoint();
  }
}
