// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PickupRoutines;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.commands.ExtendArm.SetExtArmGoal;
import frc.robot.commands.ExtendArm.WaitExtendAtTarget;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.LiftArm.SetLiftGoal;
import frc.robot.commands.LiftArm.WaitLiftAtTarget;
import frc.robot.commands.TeleopRoutines.RetractExtPositionLiftWrist;
import frc.robot.commands.Wrist.SetWristGoal;
import frc.robot.commands.Wrist.WaitWristAtTarget;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem.presetExtArmDistances;
import frc.robot.subsystems.GameHandlerSubsystem.gamePiece;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.presetIntakeSpeeds;
import frc.robot.subsystems.LiftArmSubsystem.presetLiftAngles;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystem.presetWristAngles;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundIntake extends SequentialCommandGroup {
    /** Creates a new GroundIntake. */
    public GroundIntake(LiftArmSubsystem lift, WristSubsystem wrist, ExtendArmSubsystem extend, IntakeSubsystem intake,
            gamePiece type) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand() extend, Intake);
        // assumes start from travel position
        addCommands(

                new ParallelCommandGroup(

                        new ConditionalCommand(

                                new SetLiftGoal(lift, presetLiftAngles.PICKUP_CONE_GROUND.getAngleRads()),

                                new SetLiftGoal(lift, presetLiftAngles.PICKUP_CUBE_GROUND.getAngleRads()),

                                () -> type == gamePiece.CONE),

                        new ConditionalCommand(
                                
                                new SetWristGoal(wrist, presetWristAngles.PICKUP_CONE_GROUND.getAngleRads()),

                                new SetWristGoal(wrist, presetWristAngles.PICKUP_CUBE_GROUND.getAngleRads()),

                                () -> type == gamePiece.CONE)),

                new WaitLiftAtTarget(lift, .25),

                new WaitWristAtTarget(wrist, .25),

                new ConditionalCommand(
                        new SetExtArmGoal(extend, presetExtArmDistances.PICKUP_CONE_GROUND.getDistance()),

                        new SetExtArmGoal(extend, presetExtArmDistances.PICKUP_CUBE_GROUND.getDistance()),

                        () -> type == gamePiece.CONE),

                new ConditionalCommand(

                        new RunIntake(intake,
                                presetIntakeSpeeds.PICKUP_CONE
                                        .getSpeed()),

                        new RunIntake(intake,
                                presetIntakeSpeeds.PICKUP_CUBE
                                        .getSpeed()),

                        () -> type == gamePiece.CONE),

                new ConditionalCommand(

                        new SetExtArmGoal(extend,
                                ExtendArmConstants.intakeConstraints,
                                presetExtArmDistances.PICKUP_CONE_GROUND
                                        .getDistance()),

                        new SetExtArmGoal(extend,
                                ExtendArmConstants.intakeConstraints,
                                presetExtArmDistances.PICKUP_CUBE_GROUND
                                        .getDistance()),

                        () -> type == gamePiece.CONE),

                new WaitExtendAtTarget(extend, .25),

                new GetPieceAtIntake(intake, type),

                new SetExtArmGoal(extend, ExtendArmConstants.extendArmConstraints,
                        presetExtArmDistances.HOME.getDistance()),

                new RetractExtPositionLiftWrist(lift, extend, wrist, true));

    }
}