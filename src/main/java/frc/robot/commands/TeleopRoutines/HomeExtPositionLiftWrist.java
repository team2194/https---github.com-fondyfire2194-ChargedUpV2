// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ExtendArm.RunExtArmProfile;
import frc.robot.commands.LiftArm.RunLiftArmProfile;
import frc.robot.commands.Wrist.RunWristProfile;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HomeExtPositionLiftWrist extends SequentialCommandGroup {
  /** Creates a new ArmsHomeSequence. */
  public HomeExtPositionLiftWrist(LiftArmSubsystem lift, ExtendArmSubsystem ext,

      WristSubsystem wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        new ParallelCommandGroup(

            new RunExtArmProfile(ext, ExtendArmConstants.extendArmConstraints, 0),
            new RunLiftArmProfile(lift,LiftArmConstants.liftArmConstraints, 0),
            new RunWristProfile(wrist,WristConstants.wristConstraints, 120))

    );
  }
}