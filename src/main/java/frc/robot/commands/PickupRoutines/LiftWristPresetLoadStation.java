// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PickupRoutines;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.LiftArm.SetLiftGoal;
import frc.robot.commands.LiftArm.WaitLiftClearForWrist;
import frc.robot.commands.Wrist.SetWristGoal;
import frc.robot.commands.Wrist.WaitWristAtTarget;
import frc.robot.subsystems.GameHandlerSubsystem.gamePiece;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.LiftArmSubsystem.presetLiftAngles;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystem.presetWristAngles;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LiftWristPresetLoadStation extends SequentialCommandGroup {
        /** Creates a new GroundIntake. */
        private gamePiece type;

        public LiftWristPresetLoadStation(LiftArmSubsystem lift, WristSubsystem wrist, IntakeSubsystem intake,
                        GameHandlerSubsystem ghs) {
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand() extend, Intake);
                // assumes start from travel position
                type = ghs.getGamePiecetype();

                addCommands(

                                new SetIntakePieceType(intake, type),

                                new ConditionalCommand(

                                                new SetLiftGoal(lift,
                                                                presetLiftAngles.PICKUP_CONE_LOAD_STATION
                                                                                .getInches()),

                                                new SetLiftGoal(lift,
                                                                presetLiftAngles.PICKUP_CUBE_LOAD_STATION
                                                                                .getInches()),

                                                () -> type == gamePiece.CONE),

                                // new WaitCommand(1),

                                // new WaitLiftClearForWrist(lift, 50),

                                new ConditionalCommand(

                                                new SetWristGoal(wrist,
                                                                presetWristAngles.PICKUP_CONE_LOAD_STATION
                                                                                .getAngleRads()),

                                                new SetWristGoal(wrist,
                                                                presetWristAngles.PICKUP_CUBE_LOAD_STATION
                                                                                .getAngleRads()),

                                                () -> type == gamePiece.CONE));

                // new WaitCommand(1),

                // new WaitWristAtTarget(wrist, 2));

        }
}