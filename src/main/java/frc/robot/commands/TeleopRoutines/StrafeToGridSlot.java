// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DeliverRoutines.GetSlotValues;
import frc.robot.commands.swerve.Test.MessageCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.utils.TrajectoryFactory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StrafeToGridSlot extends SequentialCommandGroup {
  /** Creates a new TrajToLoadSequence. */
  public StrafeToGridSlot(DriveSubsystem drive, TrajectoryFactory tf, GameHandlerSubsystem ghs, LimelightVision llv,
      IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        new EnsureDriveStartFromVision(drive, tf),

        new MessageCommand("Step1"),
        new InstantCommand(() -> drive.inhibitVision = true),
        new GetSlotValues(drive, ghs, llv, intake),
        new MessageCommand("3"),
        new RotateToAngle(drive, 180),

        new ConditionalCommand(tf.positionToBlueSlotCommand(), tf.positionToRedSlotCommand(),
            () -> DriverStation.getAlliance() == Alliance.Blue),
        new MessageCommand("Step3"));
  }
}
