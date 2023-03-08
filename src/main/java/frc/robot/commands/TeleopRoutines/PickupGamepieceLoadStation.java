// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ExtendArm.SetExtArmGoal;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Wrist.SetWristGoal;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem.presetExtArmDistances;
import frc.robot.subsystems.GameHandlerSubsystem.gamePiece;
import frc.robot.subsystems.IntakeSubsystem.presetIntakeSpeeds;
import frc.robot.subsystems.WristSubsystem.presetWristAngles;

public class PickupGamepieceLoadStation extends SequentialCommandGroup {
        /** Creates a new ArmsHomeSequence. */

        /** Creates a new DeliverGamepiece. */

        public PickupGamepieceLoadStation(ExtendArmSubsystem ext, WristSubsystem wrist, LiftArmSubsystem lift,

                        gamePiece type, IntakeSubsystem intake, presetIntakeSpeeds speed) {
                // Use addRequirements() here to declare subsystem dependencies.

                ;
                addCommands(

                                new ParallelCommandGroup(

                                                new ConditionalCommand(

                                                                new RunIntake(intake,
                                                                                presetIntakeSpeeds.PICKUP_CONE
                                                                                                .getSpeed()),

                                                                new RunIntake(intake,
                                                                                presetIntakeSpeeds.PICKUP_CUBE
                                                                                                .getSpeed()),

                                                                () -> type == gamePiece.CONE),

                                                new ConditionalCommand(

                                                                new SetExtArmGoal(ext,
                                                                                ExtendArmConstants.intakeConstraints,
                                                                                presetExtArmDistances.PICKUP_CONE_LOAD_STATION
                                                                                                .getDistance()),

                                                                new SetExtArmGoal(ext,
                                                                                ExtendArmConstants.intakeConstraints,
                                                                                presetExtArmDistances.PICKUP_CUBE_LOAD_STATION
                                                                                                .getDistance()),

                                                                () -> type == gamePiece.CONE)),

                                new GetPieceAtIntake(intake, type),

                                new ConditionalCommand(

                                                new SetWristGoal(wrist, WristConstants.wristConstraints,
                                                                presetWristAngles.PICKUP_CONE_LOAD_STATION
                                                                                .getAngleRads()),

                                                new SetWristGoal(wrist, WristConstants.wristConstraints,
                                                                presetWristAngles.PICKUP_CUBE_LOAD_STATION
                                                                                .getAngleRads()),

                                                () -> type == gamePiece.CONE),

                                new SetExtArmGoal(ext, ExtendArmConstants.extendArmConstraints,
                                                presetExtArmDistances.HOME.getDistance()),

                                new HomeExtPositionLiftWrist(lift, ext, wrist)

                );
        }
}
