// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PickupRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ExtendArm.SetExtArmGoal;
import frc.robot.commands.LiftArm.SetLiftGoal;
import frc.robot.commands.LiftArm.WaitLiftAtTarget;
import frc.robot.commands.Wrist.SetWristGoal;
import frc.robot.commands.Wrist.WaitWristAtTarget;
import frc.robot.subsystems.ExtendArmSubsystem;
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
public class GroundIntakeTippedConePositions extends SequentialCommandGroup {
        /** Creates a new GroundIntake. */
        public GroundIntakeTippedConePositions(LiftArmSubsystem lift, WristSubsystem wrist, ExtendArmSubsystem extend,
                        IntakeSubsystem intake) {
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand() extend, Intake);
                // assumes start from travel position
                addCommands(

                                new SetIntakePieceType(intake, gamePiece.CONE),

                                new SetLiftGoal(lift, presetLiftAngles.PICKUP_TIPPED_CONE_GROUND.getInches()),

                                new SetWristGoal(wrist, presetWristAngles.PICKUP_TIPPED_CONE_GROUND.getAngleRads()),

                                new WaitCommand(2),

                                new WaitLiftAtTarget(lift, .25),

                                new WaitWristAtTarget(wrist, .25),

                                new SetExtArmGoal(extend,
                                                presetExtArmDistances.PICKUP_TIPPED_CONE_GROUND.getDistance()),

                                new WaitCommand(2),

                                new WaitLiftAtTarget(lift, .25));

        }
}