// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PPConstants;
import frc.robot.commands.Auto.DoNothing;
import frc.robot.commands.DeliverRoutines.DeliverSelectedPieceToSelectedTarget;
import frc.robot.commands.DeliverRoutines.GetDeliverAngleSettings;
import frc.robot.commands.TeleopRoutines.BalanceRobot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

/** Add your docs here. */
public class AutoFactory {

    SwerveAutoBuilder autoBuilder;

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

    public AutoFactory(DriveSubsystem drive, GameHandlerSubsystem ghs, LiftArmSubsystem lift, ExtendArmSubsystem extend,
            WristSubsystem wrist, IntakeSubsystem intake) {

        // eventMap.put("deliverconehigh", deliverCone(nodeRow.REAR));
        // eventMap.put("pickupcube", pickupCube());

        m_drive = drive;

        m_ghs = ghs;

        m_lift = lift;

        m_extend = extend;

        m_wrist = wrist;

        m_intake = intake;

        m_autoChooser.setDefaultOption("Do Nothing", 0);

        m_autoChooser.addOption("Balance", 1);

        m_autoChooser.addOption("DriveOverCharge", 2);

        m_autoChooser.addOption("DriveOver+Balance",3);

        m_autoChooser.addOption("Clear Zone", 4);


        m_pieceLevelChooser.setDefaultOption("Top", 0);

        m_pieceLevelChooser.addOption("Mid", 1);

        m_startLocationChooser.setDefaultOption("LeftCube", 0);
        m_startLocationChooser.addOption("LeftPipe", 1);
        m_startLocationChooser.addOption("CoopLeftPipe", 2);
        m_startLocationChooser.addOption("CoopCube", 3);
        m_startLocationChooser.addOption("CoopRightPipe", 4);
        m_startLocationChooser.addOption("RightPipe", 5);
        m_startLocationChooser.addOption("RightCube", 6);
        m_startLocationChooser.addOption("LeftHybridPipe", 7);
        m_startLocationChooser.addOption("RightHybridPipe", 8);

    }

    public Command getAutonomousCommand() {

        int sel = m_startLocationChooser.getSelected();

        m_ghs.setActiveDropByNumber(sel + 1);

        int level = m_pieceLevelChooser.getSelected();

        m_ghs.setDropOffLevelByNumber(level);

        startTime = Timer.getFPGATimestamp();

        skipPathGroup = false;

        switch (m_autoChooser.getSelected()) {

            case 0:

                skipPathGroup = true;

                new DoNothing();

                break;

            case 1:

                new SequentialCommandGroup(

                        new GetDeliverAngleSettings(m_lift, m_extend, m_wrist, m_intake, m_ghs),

                        new DeliverSelectedPieceToSelectedTarget(m_lift, m_extend, m_wrist, m_intake, m_ghs),

                        new BalanceRobot(m_drive));

                break;

            case 2:

                break;

            case 3:

                break;

            default:

                break;

        }
        if (!skipPathGroup)

            autonomousCommand = autoBuilder.fullAuto(pathGroup)
                    .andThen(() -> m_drive.drive(0, 0, 0));
        ;

        return autonomousCommand;

    }

}
