// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PickupRoutines;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ExtendArm.PositionProfileExtendArm;
import frc.robot.commands.ExtendArm.SetExtArmGoal;
import frc.robot.commands.LiftArm.PositionProfileLiftInches;
import frc.robot.commands.LiftArm.SetLiftGoal;
import frc.robot.commands.LiftArm.WaitLiftAtTarget;
import frc.robot.commands.Wrist.PositionProfileWrist;
import frc.robot.commands.Wrist.SetWristGoal;
import frc.robot.commands.Wrist.WaitWristAtTarget;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem.presetExtArmDistances;
import frc.robot.subsystems.GameHandlerSubsystem.gamePiece;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.LiftArmSubsystem.presetLiftAngles;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystem.presetWristAngles;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundIntakePositions extends SequentialCommandGroup {
        /** Creates a new GroundIntake. */
        private gamePiece type;
        public GroundIntakePositions(LiftArmSubsystem lift, WristSubsystem wrist, ExtendArmSubsystem extend,
                        IntakeSubsystem intake,
                        GameHandlerSubsystem ghs) {
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand() extend, Intake);
                // assumes start from travel position
                type = ghs.getGamePiecetype();
                addCommands(

                                new ParallelRaceGroup(

                                                new ParallelCommandGroup(

                                                                new ConditionalCommand(

                                                                                new SetLiftGoal(lift,
                                                                                                presetLiftAngles.PICKUP_CONE_GROUND
                                                                                                                .getInches()),

                                                                                new SetLiftGoal(lift,
                                                                                                presetLiftAngles.PICKUP_CUBE_GROUND
                                                                                                                .getInches()),

                                                                                () -> type == gamePiece.CONE),

                                                                new PositionProfileWrist(wrist, lift),
                                                                new PositionProfileExtendArm(extend, lift),
                                                                new PositionProfileLiftInches(lift)),

                                                new SequentialCommandGroup(

                                                                new SetIntakePieceType(intake, type),

                                                                new WaitCommand(.1),

                                                                new WaitLiftAtTarget(lift, .25,.25),

                                                                new ConditionalCommand(

                                                                                new SetWristGoal(wrist,
                                                                                                presetWristAngles.PICKUP_CONE_GROUND
                                                                                                                .getAngleRads()),

                                                                                new SetWristGoal(wrist,
                                                                                                presetWristAngles.PICKUP_CUBE_GROUND
                                                                                                                .getAngleRads()),

                                                                                () -> type == gamePiece.CONE),

                                                                new WaitWristAtTarget(wrist, .25,2),

                                                                new ConditionalCommand(
                                                                        
                                                                                new SetExtArmGoal(extend,
                                                                                                presetExtArmDistances.PICKUP_CONE_GROUND
                                                                                                                .getDistance()),

                                                                                new SetExtArmGoal(extend,
                                                                                                presetExtArmDistances.PICKUP_CUBE_GROUND
                                                                                                                .getDistance()),

                                                                                () -> type == gamePiece.CONE),

                                                                new WaitCommand(.2),

                                                                new WaitLiftAtTarget(lift, .25,.5))));

        }
}