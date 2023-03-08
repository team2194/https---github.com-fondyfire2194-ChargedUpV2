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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PPConstants;
import frc.robot.commands.Auto.DoNothing;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class AutoFactory {

    SwerveAutoBuilder autoBuilder;

    List<PathPlannerTrajectory> pathGroup;

    public Command autonomousCommand = new DoNothing();

    public final SendableChooser<Integer> m_autoChooser = new SendableChooser<Integer>();

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

    

    public AutoFactory(DriveSubsystem drive) {

       // eventMap.put("deliverconehigh", deliverCone(nodeRow.REAR));
     //   eventMap.put("pickupcube", pickupCube());

        m_drive = drive;

        // m_turnArm = turnArm;

        // m_linArm = linArm;

        // Create the AutoBuilder. This only needs to be created once when robot code
        // starts,
        // not every time you want to create an auto command. A good place to put this
        // is
        // in RobotContainer along with your subsystems.

        autoBuilder = new SwerveAutoBuilder(
                m_drive::getEstimatedPosition, // null,
                m_drive::resetOdometry, // null,
                DriveConstants.m_kinematics, // null,

                new PIDConstants(PPConstants.kPXController, PPConstants.kIXController, PPConstants.kDXController),

                new PIDConstants(PPConstants.kPThetaController, PPConstants.kIThetaController,
                        PPConstants.kDThetaController),

                m_drive::setModuleStates, // Module states consumer used to output to the drive subsystem
                eventMap,
                m_drive);

        m_autoChooser.setDefaultOption("Do Nothing", 0);

         m_autoChooser.addOption("Drive Forward", 1);

         m_autoChooser.addOption("Drive Straight", 2);

    }

    public Command getAutonomousCommand() {

        startTime = Timer.getFPGATimestamp();

        skipPathGroup = false;

        switch (m_autoChooser.getSelected()) {

            case 0:

                skipPathGroup = true;

                autonomousCommand = new DoNothing();

                break;

            case 1:

                skipPathGroup = true;

                PathPlannerTrajectory driveForward = PathPlanner.loadPath("DriveForward", 4, 3);

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
