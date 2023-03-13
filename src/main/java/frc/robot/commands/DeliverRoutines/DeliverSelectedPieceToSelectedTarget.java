// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeliverRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ExtendArm.SetExtArmGoal;
import frc.robot.commands.ExtendArm.WaitExtendAtTarget;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.LiftArm.SetLiftGoal;
import frc.robot.commands.LiftArm.WaitLiftAtTarget;
import frc.robot.commands.TeleopRoutines.RetractWristExtendLift;
import frc.robot.commands.Wrist.SetWristGoal;
import frc.robot.commands.Wrist.WaitWristAtTarget;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeliverSelectedPieceToSelectedTarget extends SequentialCommandGroup {
  /** Creates a new DeliverSelectedPieceToSelectedTarget. */
  public DeliverSelectedPieceToSelectedTarget(LiftArmSubsystem lift, ExtendArmSubsystem extend, WristSubsystem wrist,
      IntakeSubsystem intake, GameHandlerSubsystem ghs) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        new GetDeliverAngleSettings(lift, extend, wrist, intake, ghs),

        new SetLiftGoal(lift, LiftArmConstants.liftArmConstraints, lift.deliverAngleRads),

        new SetWristGoal(wrist, WristConstants.wristConstraints, wrist.deliverAngleRads),

        new WaitLiftAtTarget(lift, .25),

        new WaitWristAtTarget(wrist, .24),

        new SetExtArmGoal(extend, ExtendArmConstants.deliverConstraints, extend.deliverDistance),

        new WaitExtendAtTarget(extend, .24),

        new RunIntake(intake, intake.deliverSpeed)

            .until(() -> !intake.selectSensor(ghs.getGamePiecetype()))

            .andThen(new WaitCommand(.2))

            .andThen(new StopIntake(intake)),

        new RetractWristExtendLift(lift, extend, wrist, true)

    );
  }
}
