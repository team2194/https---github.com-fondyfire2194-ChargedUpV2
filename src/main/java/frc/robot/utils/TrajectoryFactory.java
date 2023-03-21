// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.io.File;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TeleopRoutines.StrafeToBlueFieldY;
import frc.robot.commands.TeleopRoutines.StrafeToRedFieldY;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem.fieldTagsBlue;
import frc.robot.subsystems.GameHandlerSubsystem.fieldTagsRed;

/** Add your docs here. */
public class TrajectoryFactory {

    public SendableChooser<String> ppTrajChooser = new SendableChooser<String>();

    private DriveSubsystem m_drive;

    private GameHandlerSubsystem m_ghs;

    private FieldSim m_fs;

    private boolean tune;

    public boolean run;

    public boolean runTagTraj;

    public Pose2d activeTagPose;

    public Pose2d endPose = new Pose2d();

    public double loadStationYOffset = .25;

    public Translation2d leftTranslation = new Translation2d(1.25, loadStationYOffset);

    public Translation2d rightTranslation = new Translation2d(1.25, -loadStationYOffset);

    private Translation2d activeTranslation;

    public Rotation2d endRotation = Rotation2d.fromDegrees(180);

    public boolean showSelected;

    public boolean showLoadTraj;

    public Pose2d startLoadPose;

    PathPlannerTrajectory selTraj;

    public TrajectoryFactory(DriveSubsystem drive, FieldSim fs, GameHandlerSubsystem ghs) {

        m_drive = drive;
        m_fs = fs;
        m_ghs = ghs;

        ppTrajChooser.setDefaultOption("BackUpCenterPath", "BackUpCenter");
        ppTrajChooser.addOption("BackUpLeftCenterPath", "BackUpLeftCenter");
        ppTrajChooser.addOption("BackUpLeftShelfPath", "BackUpLeftShelf");
        ppTrajChooser.addOption("BackUpRightCenterPath", "BackUpRightCenter");
        ppTrajChooser.addOption("BackUpRightShelfPath", "BackUpRightShelf");

        ppTrajChooser.addOption("PushCubeCenterPath", "PushCubeCenter");
        ppTrajChooser.addOption("PushCubeLeftShelfPath", "PushCubeLeftShelf");
        ppTrajChooser.addOption("PushCubeRighShelfPath", "PushCubeRightShelf");

        createSelectedTrajectory(2, 2, true);

    }

    public String getSelectedTrajectoryName() {
        return ppTrajChooser.getSelected();
    }

    public PathPlannerTrajectory getPathPlannerTrajectory(String pathName, double maxvel, double maxaccel,
            boolean reversed) {
        // checkFileExists(pathName);
        PathPlannerTrajectory ppTrajectory = PathPlanner.loadPath(pathName, maxvel, maxaccel, reversed);
        return ppTrajectory;
    }

    public boolean checkFileExists(String name) {
        File deployDirectory = Filesystem.getDeployDirectory();
        String pathName = deployDirectory.toString() + "\\" + name + ".path";

        // SmartDashboard.putString("Name", pathName);
        File f = new File(pathName);

        // SmartDashboard.putBoolean("Exists", f.exists());// && !f.isDirectory()));
        return f.exists() && !f.isDirectory();
    }

    public void createSelectedTrajectory(double maxvel, double maxaccel, boolean isFirstPath) {

        String selectedName = getSelectedTrajectoryName();
        // SmartDashboard.putString("SELName", selectedName);
        selTraj = getPathPlannerTrajectory(selectedName, maxvel, maxaccel, isFirstPath);
    }

    public Command runSelectedTrajectory() {
        return followTrajectoryCommand(selTraj, true);
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {

                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        PathPlannerTrajectory transformed = PathPlannerTrajectory.transformTrajectoryForAlliance(traj,
                                DriverStation.getAlliance());
                       // m_drive.resetGyro();
                        m_drive.resetOdometry(transformed.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(

                        traj,

                        m_drive::getEstimatedPosition, // Pose supplier

                        m_drive.m_kinematics, // SwerveDriveKinematics

                        m_drive.getXPID(),

                        m_drive.getYPID(),

                        m_drive.getThetaPID(),

                        m_drive::setModuleStates, // Module states consumer

                        true,

                        m_drive // Requires this drive subsystem
                ),

                new InstantCommand(() -> m_drive.stopModules()),

                new ParallelCommandGroup(

                        new InstantCommand(() -> run = false),

                        new InstantCommand(() -> m_drive.setInhibitVisionCorrection(false)),

                        new InstantCommand(() -> runTagTraj = false)));
    }

    public void setActiveTagPose(Pose2d pose) {

        activeTagPose = pose;
    }

    public void clearTrajectory() {
        PathPlannerTrajectory traj = new PathPlannerTrajectory();
        m_fs.getField2d().getObject("Traj").setTrajectory(traj);

    }

    public void periodic() {

    }

    public void setLeftPickup(boolean leftPickup) {

        activeTranslation = rightTranslation;

        if (leftPickup)

            activeTranslation = leftTranslation;
    }

    public void runSelected() {
        String name = getSelectedTrajectoryName();
        PathPlannerTrajectory traj = getPathPlannerTrajectory(name, 2,
                2, false);
        followTrajectoryCommand(traj, true).schedule();
    }

    public void runLoad(boolean left) {

        if (DriverStation.getAlliance() == Alliance.Blue) {

            setActiveTagPose(fieldTagsBlue.BLUE_LOAD.getPose());
        }

        else {

            setActiveTagPose(fieldTagsRed.RED_LOAD.getPose());

        }

        setLeftPickup(left);

        PathPlannerTrajectory traj = getTrajToLoadTag(activeTagPose, 2, 2);

        followTrajectoryCommand(traj, true).schedule();

    }

    public void runPosiitonToGridSlot() {
        if (DriverStation.getAlliance() == Alliance.Blue) {
            positionToBlueSlotCommand().schedule();
        } else {
            positionToRedSlotCommand().schedule();
        }
    }

    public PathPlannerTrajectory getTrajToLoadTag(Pose2d tagPose, double maxvel, double maxAccel) {

        Translation2d tagTranslation = new Translation2d(tagPose.getX(), tagPose.getY());

        Rotation2d tagRotation = tagPose.getRotation();

        Translation2d modifiedTranslation = tagTranslation.minus(activeTranslation);

        Rotation2d modifiedRotation = tagRotation.plus(endRotation);

        endPose = new Pose2d(modifiedTranslation, modifiedRotation);

        PathPlannerTrajectory trajt = PathPlanner.generatePath(

                new PathConstraints(maxvel, maxAccel),

                new PathPoint(startLoadPose.getTranslation(), startLoadPose.getRotation()), // position, heading

                new PathPoint(endPose.getTranslation(), endPose.getRotation()) // position,
                                                                               // heading

        );

        return trajt;
    }

    public void doSelectedTrajectory() {
        runSelectedTrajectory().schedule();
    }

    public PathPlannerTrajectory getSimpleTraj() {

        // Simple path without holonomic rotation. Stationary start/end. Max velocity of
        // 4 m/s and max accel of 3 m/s^2

        PathPlannerTrajectory traj1 = PathPlanner.generatePath(

                new PathConstraints(2, 2),

                new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0)), // position, heading

                new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45)) // position, heading
        );
        return traj1;
    }

    public PathPlannerTrajectory getSimpleTrajWithHolonomic() {

        // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4
        // m/s and max accel of 3 m/s^2
        PathPlannerTrajectory traj2 = PathPlanner.generatePath(
                new PathConstraints(4, 3),

                new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position,

                new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-90)) // position,

        );

        return traj2;
    }

    // More complex path with holonomic rotation. Non-zero starting velocity of 2
    // m/s. Max velocity of 4 m/s and max accel of 3 m/s^2

    public PathPlannerTrajectory getNonZeroStartTraj() {

        PathPlannerTrajectory traj3 = PathPlanner.generatePath(

                new PathConstraints(4, 3),

                new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 2), // position,

                new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-90)), // position,

                new PathPoint(new Translation2d(5.0, 3.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-30)) // position,

        );
        return traj3;
    }

    public Command positionToBlueSlotCommand() {
        return new StrafeToBlueFieldY(m_drive, m_ghs);

    }

    public Command positionToRedSlotCommand() {
        return new StrafeToRedFieldY(m_drive, m_ghs);

    }

    public boolean getAllianceBlue() {
        return (DriverStation.getAlliance() == Alliance.Blue);
    }

}
