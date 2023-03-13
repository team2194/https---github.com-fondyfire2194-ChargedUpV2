// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ExtendArm.SetExtArmGoal;
import frc.robot.commands.ExtendArm.WaitExtendAtTarget;
import frc.robot.commands.LiftArm.SetLiftGoal;
import frc.robot.commands.LiftArm.WaitLiftAtTarget;
import frc.robot.commands.Wrist.SetWristGoal;
import frc.robot.commands.Wrist.WaitWristAtTarget;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem.presetExtArmDistances;
import frc.robot.subsystems.LiftArmSubsystem.presetLiftAngles;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystem.presetWristAngles;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetractWristExtendLift extends SequentialCommandGroup {
        /** Creates a new ArmsHomeSequence. */
        public RetractWristExtendLift(LiftArmSubsystem lift, ExtendArmSubsystem ext,

                        WristSubsystem wrist, boolean travel) {
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand());
                addCommands(

                                new ConditionalCommand(

                                                new SetWristGoal(wrist, WristConstants.wristConstraints,
                                                                presetWristAngles.SAFE_TRAVEL
                                                                                .getAngleRads()),

                                                new SetWristGoal(wrist,
                                                                presetWristAngles.HOME.getAngleRads()),
                                                () -> travel),

                                new WaitWristAtTarget(wrist, .25),

                                new ConditionalCommand(

                                                new SetExtArmGoal(ext,
                                                                presetExtArmDistances.SAFE_TRAVEL
                                                                                .getDistance()),

                                                new SetExtArmGoal(ext,
                                                                presetExtArmDistances.HOME
                                                                                .getDistance()),
                                                () -> travel),

                                new WaitExtendAtTarget(ext, .25),

                                new ConditionalCommand(

                                                new SetLiftGoal(lift,
                                                                presetLiftAngles.TRAVEL
                                                                                .getAngleRads()),

                                                new SetLiftGoal(lift,
                                                                presetLiftAngles.SAFE_HOME
                                                                                .getAngleRads()),
                                                () -> travel),

                                new WaitLiftAtTarget(lift, .25)

                );

        }
}
