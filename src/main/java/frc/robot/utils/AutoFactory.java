// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auto.DoNothing;
import frc.robot.commands.DeliverRoutines.DeliverPiecePositions;
import frc.robot.commands.DeliverRoutines.GetDeliverAngleSettings;
import frc.robot.commands.TeleopRoutines.DriveandBalanceRobot;
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

    public final SendableChooser<Integer> m_startLocationChooser = new SendableChooser<Integer>();

    public final SendableChooser<Integer> m_pieceLevelChooser = new SendableChooser<Integer>();

    private boolean skipPathGroup;

    private double startTime;

    private enum nodeRow {
        REAR,
        MID,
        FRONT
    }

    /**
     * Event map for auto paths
     */
    HashMap<String, Command> eventMap = new HashMap<>();

    private DriveSubsystem m_drive;

    private GameHandlerSubsystem m_ghs;

    private LiftArmSubsystem m_lift;

    private ExtendArmSubsystem m_extend;

    private WristSubsystem m_wrist;

    private IntakeSubsystem m_intake;

    private TrajectoryFactory m_tf;

    private PathPlannerTrajectory traj1;

    private PathPlannerTrajectory traj2;

    private Transform2d center2LeftHybrid = new Transform2d(new Translation2d(0, 1), new Rotation2d());

    private Transform2d center2RightHybrid = new Transform2d();

    private Transform2d leftHybrid2RightHybrid = new Transform2d();

    private Transform2d centerCoopToLeftCoop = new Transform2d();

    private Transform2d centerCoopToRightCoop = new Transform2d();

    private Command command1 = new DoNothing();

    private Command command2= new DoNothing();

    public AutoFactory(DriveSubsystem drive, GameHandlerSubsystem ghs, LiftArmSubsystem lift, ExtendArmSubsystem extend,
            WristSubsystem wrist, IntakeSubsystem intake, TrajectoryFactory tf) {

        // eventMap.put("deliverconehigh", deliverCone(nodeRow.REAR));
        // eventMap.put("pickupcube", pickupCube());

        m_drive = drive;

        m_ghs = ghs;

        m_lift = lift;

        m_extend = extend;

        m_wrist = wrist;

        m_intake = intake;

        m_tf = tf;

        m_autoChooser.setDefaultOption("Do Nothing", 0);

        m_autoChooser.addOption("PushCubeClearZone", 1);

        m_autoChooser.addOption("DeliverClearZone", 2);

        m_autoChooser.addOption("PushCubeDriveOver", 3);

        m_autoChooser.addOption("DeliverDriveOver", 4);

        m_autoChooser.addOption("PushCubeBalance", 5);

        m_autoChooser.addOption("DeliverBalance", 6);

        m_pieceLevelChooser.setDefaultOption("Top", 0);
        m_pieceLevelChooser.addOption("Mid", 1);

        m_startLocationChooser.setDefaultOption("CoopLeftPipe", 0);
        m_startLocationChooser.addOption("CoopCube", 1);
        m_startLocationChooser.addOption("CoopRightPipe", 2);

        m_startLocationChooser.addOption("LeftHybridPipe", 3);
        m_startLocationChooser.addOption("RightHybridPipe", 4);

    }

    public Command getAutonomousCommand() {

        int sel = m_startLocationChooser.getSelected();

        int ac = m_autoChooser.getSelected();

        if (sel <= 2) {

            m_ghs.setActiveDropByNumber(sel + 1);
        }

        int level = m_pieceLevelChooser.getSelected();

        startTime = Timer.getFPGATimestamp();

        switch (ac) {

            // just sit there

            case 0:

                break;

            // push cube clear zone
            case 1:

                switch (sel) {
                    // left hybrid start
                    case 7:

                        traj1 = m_tf.getPathPlannerTrajectory("PushConeCenter", 3, 3, false);
                        traj1.transformBy(center2LeftHybrid);
                        traj2 = m_tf.getPathPlannerTrajectory("BackupLeftHybrid", 3, 3, false);

                        break;
                    // right hybrid start
                    case 8:

                        traj1 = m_tf.getPathPlannerTrajectory("PushConeCenter", 3, 3, false);
                        traj1.transformBy(center2RightHybrid);
                        traj2 = m_tf.getPathPlannerTrajectory("BackUpLeftHybrid", 3, 3, false);
                        traj2.transformBy(leftHybrid2RightHybrid);

                        break;

                    default:

                        break;
                }

                break;

            // deliver clear zone

            case 2:

                command1 = getDeliverValuesAndExecute();

                break;

            case 3:

                break;

            case 4:

                command1 = getDeliverValuesAndExecute();

                break;

            case 5:

                command1 = getDeliverValuesAndExecute();

                break;

            case 6:

                command1 = getDeliverValuesAndExecute();

                break;

            default:

                break;

        }

        autonomousCommand = new SequentialCommandGroup(command1, command2);

        return autonomousCommand;

    }

    private Command getDeliverValuesAndExecute() {

        return new SequentialCommandGroup(

                new GetDeliverAngleSettings(m_lift, m_extend, m_wrist, m_intake, true),

                new DeliverPiecePositions(m_lift, m_extend, m_wrist, m_intake));

    }

    private Command doTrajectory(PathPlannerTrajectory traj) {
        return m_tf.followTrajectoryCommand(traj, true);

    }

}
