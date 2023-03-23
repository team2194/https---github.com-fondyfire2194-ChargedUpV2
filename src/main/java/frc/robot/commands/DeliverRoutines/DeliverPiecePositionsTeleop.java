// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeliverRoutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ExtendArm.PositionProfileExtendArm;
import frc.robot.commands.ExtendArm.SetExtArmGoal;
import frc.robot.commands.ExtendArm.WaitExtendAtTarget;
import frc.robot.commands.LiftArm.PositionProfileLiftInches;
import frc.robot.commands.LiftArm.SetLiftGoal;
import frc.robot.commands.LiftArm.WaitLiftAtTarget;
import frc.robot.commands.TeleopRoutines.RetractWristExtendLift;
import frc.robot.commands.Wrist.PositionProfileWrist;
import frc.robot.commands.Wrist.SetWristGoal;
import frc.robot.commands.Wrist.WaitWristAtTarget;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem.presetExtArmDistances;
import frc.robot.subsystems.IntakeSubsystem.presetIntakeSpeeds;
import frc.robot.subsystems.LiftArmSubsystem.presetLiftAngles;
import frc.robot.subsystems.WristSubsystem.presetWristAngles;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeliverPiecePositionsTeleop extends SequentialCommandGroup {
    /** Creates a new DeliverSelectedPieceToSelectedTarget. */
    public DeliverPiecePositionsTeleop(LiftArmSubsystem lift, ExtendArmSubsystem extend, WristSubsystem wrist,
            IntakeSubsystem intake) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(

                new ParallelRaceGroup(

                        new ParallelCommandGroup(

                                new SetLiftGoal(lift, LiftArmConstants.liftArmFastConstraints, lift.deliverInches),

                                new SetWristGoal(wrist, WristConstants.wristFastConstraints, wrist.deliverAngleRads),

                                new PositionProfileWrist(wrist, lift),
                                new PositionProfileExtendArm(extend, lift),
                                new PositionProfileLiftInches(lift)),

                        new SequentialCommandGroup(

                                new WaitLiftAtTarget(lift, 2, 3),

                                new WaitWristAtTarget(wrist, .24, .2),

                                new SetExtArmGoal(extend, ExtendArmConstants.extendArmFastConstraints,
                                        extend.deliverDistance),

                                new WaitExtendAtTarget(extend, .3, 5))));

    }
}