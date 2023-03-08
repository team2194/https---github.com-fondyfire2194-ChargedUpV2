// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.commands.ExtendArm.RunExtArmProfile;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeliverGamepieceToGridDrop extends SequentialCommandGroup {
  /** Creates a new DeliverGamepieceToGridDrop. */
  public DeliverGamepieceToGridDrop(LiftArmSubsystem turn, ExtendArmSubsystem ext, WristSubsystem wrist,
      GameHandlerSubsystem ghs, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        new GetDeliverAngleSettings(turn, ext, wrist, ghs),

        new HomeExtPositionLiftWrist(turn, ext, wrist),

        new RunExtArmProfile(ext, ExtendArmConstants.deliverConstraints, ext.deliverDistance),

        new IntakeDeliverPiece(intake)

    );
  }
}
