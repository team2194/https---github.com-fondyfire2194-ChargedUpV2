// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auto.DoNothing;
import frc.robot.commands.DeliverRoutines.DeliverPiecePositions;
import frc.robot.commands.DeliverRoutines.EjectPieceFromIntake;
import frc.robot.commands.DeliverRoutines.GetDeliverAngleSettings;
import frc.robot.commands.TeleopRoutines.RetractWristExtendLift;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

/** Add your docs here. */
public class AutoFactory {

    // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(null, null, null, null,
    // null, null, null);

    List<PathPlannerTrajectory> pathGroup;

    public Command autonomousCommand = new DoNothing();

    public final SendableChooser<Integer> m_autoChooser = new SendableChooser<Integer>();

    public final SendableChooser<Integer> m_autoChooser1 = new SendableChooser<Integer>();

    public final SendableChooser<Integer> m_startLocationChooser = new SendableChooser<Integer>();

    public final SendableChooser<Double> m_startDelayChooser = new SendableChooser<Double>();

    private DriveSubsystem m_drive;

    private GameHandlerSubsystem m_ghs;

    private LiftArmSubsystem m_lift;

    private ExtendArmSubsystem m_extend;

    private WristSubsystem m_wrist;

    private IntakeSubsystem m_intake;

    private TrajectoryFactory m_tf;

    private PathPlannerTrajectory traj1;

    private PathPlannerTrajectory traj2;

    private Command command1 = new DoNothing();

    private Command command2 = new DoNothing();

    private Command command3 = new DoNothing();

    public String traj1name = "PushCubeCenter";

    public String traj2name = "BackUpCenter";

    private double startTime;

    public AutoFactory(DriveSubsystem drive, GameHandlerSubsystem ghs, LiftArmSubsystem lift, ExtendArmSubsystem extend,
            WristSubsystem wrist, IntakeSubsystem intake, TrajectoryFactory tf) {

        m_drive = drive;

        m_ghs = ghs;

        m_lift = lift;

        m_extend = extend;

        m_wrist = wrist;

        m_intake = intake;

        m_tf = tf;

        m_startDelayChooser.setDefaultOption("Zero Seconds", 0.);
        m_startDelayChooser.addOption("One Second", 1.);
        m_startDelayChooser.addOption("Two Seconds", 2.);
        m_startDelayChooser.addOption("Three Second", 3.);
        m_startDelayChooser.addOption("Four Seconds", 4.);
        m_startDelayChooser.addOption("Five Seconds", 4.);

        m_autoChooser.setDefaultOption("Do Nothing", 0);

        m_autoChooser.addOption("PushCube", 1);

        m_autoChooser.addOption("DeliverMid", 2);


        m_autoChooser1.setDefaultOption("Do Nothing", 0);

        m_autoChooser1.addOption("BalanceCharge", 1);

        m_autoChooser1.addOption("DriveThroughCharge", 2);

        m_autoChooser1.addOption("DriveOutZone", 3);

        m_startLocationChooser.setDefaultOption("CoopShelf", 0);

        m_startLocationChooser.addOption("CoopLeftPipe", 1);

        m_startLocationChooser.addOption("CoopRightPipe", 2);

        m_startLocationChooser.addOption("LeftShelf", 3);

        m_startLocationChooser.addOption("RightShelf", 4);

    }

    public Command getCommand1() {

        Command tempCommand = new DoNothing();

        boolean trajReqd = false;

        int startLocation = m_startLocationChooser.getSelected();

        int autoselect = m_autoChooser.getSelected();

        if (startLocation == 0 && autoselect == 1) {
            traj1name = "PushCubeCenter";

            trajReqd = true;
        }

        if (startLocation == 3 && autoselect == 1) {
            traj1name = "PushCubeLeftShelf";
            trajReqd = true;
        }
        if (startLocation == 4 && autoselect == 1) {
            traj1name = "PushCubeRightShelf";
            trajReqd = true;
        }

        if (trajReqd) {

            traj1 = m_tf.getPathPlannerTrajectory(traj1name, 2, 1, false);

            tempCommand = m_tf.followTrajectoryCommand(traj1, true);
        }

        if (autoselect == 2)
            tempCommand = getDeliverMid();


        return tempCommand;

    }

    public Command getCommand2() {

        Command tempCommand = new DoNothing();

        boolean trajReqd = false;

        int startLocation = m_startLocationChooser.getSelected();

        int autoselect1 = m_autoChooser1.getSelected();

        if (startLocation <= 2) {// any of the coop starts

            if (autoselect1 == 1) {

                tempCommand = m_drive.autoBalance();
            }

            if (startLocation == 0 && autoselect1 == 2) {

                traj2name = "BackUpCenter";

                trajReqd = true;
            }

            if (startLocation == 1 && autoselect1 == 2) {

                traj2name = "BackUpLeftCenter";

                trajReqd = true;
            }

            if (startLocation == 2 && autoselect1 == 2) {

                traj2name = "BackUpRightCenter";

                trajReqd = true;
            }
        }
        if (startLocation == 3) {
            traj2name = "BackUpLeftShelf";
            trajReqd = true;
        }

        if (startLocation == 4) {
            traj2name = "BackUpRightShelf";
            trajReqd = true;
        }

        if (trajReqd) {

            traj2 = m_tf.getPathPlannerTrajectory(traj2name, 2, 1, false);

            tempCommand = m_tf.followTrajectoryCommand(traj2, true);

        }

        return tempCommand;

    }

    public void createCommands() {
        command1 = getCommand1();
        command2 = getCommand2();
    }

    public Command getAutonomousCommand() {

        startTime = Timer.getFPGATimestamp();

        autonomousCommand = new SequentialCommandGroup(command1, command2);

        return autonomousCommand;

    }

    private Command getDeliverMid() {

        return new SequentialCommandGroup(

                new GetDeliverAngleSettings(m_lift, m_extend, m_wrist, m_intake, false),

                new DeliverPiecePositions(m_lift, m_extend, m_wrist, m_intake),

                new EjectPieceFromIntake(m_intake),

                new RetractWristExtendLift(m_lift, m_extend, m_wrist, false));

    }

    private PathPlannerTrajectory getTrajectory(String name) {

        PathPlannerTrajectory traj = m_tf.getPathPlannerTrajectory(name, 2, 2, false);

        return traj;

    }

    private Command doTrajectory(PathPlannerTrajectory traj) {
        return m_tf.followTrajectoryCommand(traj, true);

    }

}
