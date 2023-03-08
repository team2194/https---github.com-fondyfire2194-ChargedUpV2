// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.TrajectoryFactory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrajToLoadSequence extends SequentialCommandGroup {
  /** Creates a new TrajToLoadSequence. */
  public TrajToLoadSequence(DriveSubsystem drive, TrajectoryFactory tf, boolean left) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        new EnsureDriveStartFromVision(drive, tf),
        
        new InstantCommand(() -> tf.runLoad(left)));
  }
}
