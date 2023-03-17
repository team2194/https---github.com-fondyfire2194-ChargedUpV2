
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DeliverRoutines.EjectPieceFromIntake;
import frc.robot.commands.DeliverRoutines.GetDeliverAngleSettings;
import frc.robot.commands.DeliverRoutines.SetSwerveDriveTape;
import frc.robot.commands.ExtendArm.JogExtendArm;
import frc.robot.commands.ExtendArm.PositionProfileExtendArm;
import frc.robot.commands.Intake.JogIntake;
import frc.robot.commands.LiftArm.JogLiftArm;
import frc.robot.commands.LiftArm.PositionProfileLiftInches;
import frc.robot.commands.NTs.MonitorThreadExt;
import frc.robot.commands.NTs.MonitorThreadIntake;
import frc.robot.commands.NTs.MonitorThreadLift;
import frc.robot.commands.NTs.MonitorThreadWrist;
import frc.robot.commands.PickupRoutines.DetectorLoad;
import frc.robot.commands.PickupRoutines.GetPieceExpectedAtIntake;
import frc.robot.commands.PickupRoutines.GroundIntakePositions;
import frc.robot.commands.PickupRoutines.GroundIntakeTippedConePositions;
import frc.robot.commands.PickupRoutines.LiftWristPresetLoadStation;
import frc.robot.commands.TeleopRoutines.RetractWristExtendLift;
import frc.robot.commands.Wrist.JogWrist;
import frc.robot.commands.Wrist.PositionProfileWrist;
import frc.robot.commands.Wrist.RaiseLowerWrist;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.oi.RumbleCommand;
import frc.robot.oi.ShuffleboardArms;
import frc.robot.oi.ShuffleboardCompetition;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem.gamePiece;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LLDriveLinkerSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.LightStrip;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.AutoFactory;
import frc.robot.utils.LEDControllerI2C;
import frc.robot.utils.TrajectoryFactory;

public class RobotContainer {

        // The robot's subsystems
        final DriveSubsystem m_drive;

        final LiftArmSubsystem m_liftArm;// = new LiftArmSubsystem();

        final ExtendArmSubsystem m_extendArm;// = new ExtendArmSubsystem();

        final IntakeSubsystem m_intake = new IntakeSubsystem();

        final WristSubsystem m_wrist = new WristSubsystem();

        final LimelightVision m_llv;// = new LimelightVision();

        final ShuffleboardCompetition m_shc;

        final ShuffleboardArms m_sharm;

        public AutoFactory m_autoFactory;

        public TrajectoryFactory m_tf;

        public GameHandlerSubsystem m_ghs;

        public LEDControllerI2C lcI2;

        public FieldSim m_fieldSim = null;

        // The driver, codriver and arm controllers

        public CommandXboxController m_driverController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);

        public CommandXboxController m_coDriverController = new CommandXboxController(
                        OIConstants.kCoDriverControllerPort);

        final PowerDistribution m_pdp = new PowerDistribution();

        public LimelightVision m_llvis = new LimelightVision();

        public LLDriveLinkerSubsystem m_lldv;

        public LightStrip m_ls = new LightStrip(9, 60);

        public MonitorThreadExt mext;
        public MonitorThreadLift mlift;
        public MonitorThreadWrist mwrist;
        public MonitorThreadIntake mIntake;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                Pref.deleteUnused();

                Pref.addMissing();

                m_drive = new DriveSubsystem();

                m_liftArm = new LiftArmSubsystem();

                mlift = new MonitorThreadLift(m_liftArm);

                // mlift.startThread();

                m_extendArm = new ExtendArmSubsystem();

                mext = new MonitorThreadExt(m_extendArm);

                // mext.startThread();

                mwrist = new MonitorThreadWrist(m_wrist);

                // mwrist.startThread();

                mIntake = new MonitorThreadIntake(m_intake);

                // mIntake.startThread();

                m_llv = new LimelightVision();

                m_lldv = new LLDriveLinkerSubsystem(m_llv, m_drive);

                SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

                LiveWindow.disableAllTelemetry();

                m_ghs = new GameHandlerSubsystem();

                m_autoFactory = new AutoFactory(m_drive, m_ghs, m_liftArm, m_extendArm, m_wrist, m_intake, m_tf);

                m_fieldSim = new FieldSim(m_drive, m_ghs);

                m_fieldSim.initSim();

                m_tf = new TrajectoryFactory(m_drive, m_fieldSim, m_ghs);

                m_shc = new ShuffleboardCompetition(m_llv, m_drive, m_ghs, m_autoFactory,
                                m_liftArm, m_extendArm,
                                m_wrist, m_intake);

                m_sharm = new ShuffleboardArms(m_liftArm, m_extendArm, m_wrist,
                                m_intake, m_tf);

                // SmartDashboard.putData("Drive", m_drive);
                // SmartDashboard.putData("LiftArm", m_liftArm);
                // SmartDashboard.putData("Ext Arm", m_extendArm);
                // SmartDashboard.putData("Intake", m_intake);
                // SmartDashboard.putData("Wrist", m_wrist);

                // PortForwarder.add(5800, "10.21.94.11", 5800);
                // PortForwarder.add(1181, "10.21.94.11", 1181);
                // PortForwarder.add(1182, "10.21.94.11", 1182);
                // PortForwarder.add(1183, "10.21.94,11", 1183);
                // PortForwarder.add(1184, "10.21.94.11", 1184);

                // CommandScheduler.getInstance()
                // .onCommandInitialize(command -> System.out.println(command.getName() + " is
                // starting"));
                // CommandScheduler.getInstance()
                // .onCommandFinish(command -> System.out.println(command.getName() + " has
                // ended"));
                // CommandScheduler.getInstance()
                // .onCommandInterrupt(
                // command -> System.out.println(command.getName() + " was interrupted"));
                // CommandScheduler.getInstance().onCommandInitialize(
                // command -> SmartDashboard.putString("CS", command.getName() + " is
                // starting"));
                // CommandScheduler.getInstance()
                // .onCommandFinish(command -> SmartDashboard.putString("CE",
                // command.getName() + " has Ended"));
                // CommandScheduler.getInstance().onCommandInterrupt(
                // command -> SmartDashboard.putString("CE", command.getName() + "was
                // Interrupted"));

                setDefaultCommands();

                configDriverButtons();

                configCodriverButtons();

        }

        private void setDefaultCommands() {

                // m_drive.setDefaultCommand(getDriveCommand());

                m_extendArm.setDefaultCommand(new PositionProfileExtendArm(m_extendArm,
                                m_liftArm));

                m_liftArm.setDefaultCommand(new PositionProfileLiftInches(m_liftArm));

                m_wrist.setDefaultCommand(new PositionProfileWrist(m_wrist, m_liftArm));

        }

        void configDriverButtons() {

                m_driverController.leftTrigger()
                                .whileTrue(getDetectorLoad());

                m_driverController.rightTrigger().whileTrue(new SetSwerveDriveTape(m_drive, m_llv, m_ghs,
                                () -> m_driverController.getRawAxis(1)));

                m_driverController.leftBumper()
                                .onTrue(new RetractWristExtendLift(m_liftArm, m_extendArm, m_wrist, false));

                m_driverController.rightBumper().onTrue(redoGoals());

                m_driverController.a().onTrue(new EjectPieceFromIntake(m_intake).withTimeout(5));

                m_driverController.b().onTrue(new GetPieceExpectedAtIntake(m_intake).withTimeout(5));

                m_driverController.x().onTrue(Commands.runOnce(() -> m_ghs.toggleGamePieceType()));

                m_driverController.y().onTrue(getLoadSettings().withTimeout(2));

                // m_driverController.start()

                // m_driverController.back()

                m_driverController.povUp().whileTrue(new JogWrist(m_wrist, () -> -.4, m_driverController))
                                .onFalse(Commands.waitUntil(() -> m_wrist.isStopped())
                                                .andThen(Commands.runOnce(() -> m_wrist.setControllerAtPosition(),
                                                                m_wrist)));

                m_driverController.povDown().whileTrue(new JogWrist(m_wrist, () -> .4, m_driverController))
                                .onFalse(Commands.waitUntil(() -> m_wrist.isStopped())
                                                .andThen(Commands.runOnce(() -> m_wrist.setControllerAtPosition(),
                                                                m_wrist)));

                m_driverController.povLeft().whileTrue(new JogExtendArm(m_extendArm, () -> -.4, m_driverController))
                                .onFalse(Commands.waitUntil(() -> m_extendArm.isStopped())
                                                .andThen(Commands.runOnce(() -> m_extendArm.setControllerAtPosition(),
                                                                m_extendArm)));

                m_driverController.povRight().whileTrue(new JogExtendArm(m_extendArm, () -> -.4, m_driverController))
                                .onFalse(Commands.waitUntil(() -> m_extendArm.isStopped())
                                                .andThen(Commands.runOnce(() -> m_extendArm.setControllerAtPosition(),
                                                                m_extendArm)));

        }

        private void configCodriverButtons() {

                m_coDriverController.leftBumper().whileTrue(getJogLiftArmCommand(m_coDriverController))

                                .onFalse(Commands.waitUntil(() -> m_liftArm.isStopped())

                                                .andThen(Commands.runOnce(() -> m_liftArm.setControllerAtPosition(),
                                                                m_liftArm)));

                m_coDriverController.leftTrigger().whileTrue(getJogExtendArmCommand(m_coDriverController))

                                .onFalse(Commands.waitUntil(() -> m_extendArm.isStopped())

                                                .andThen(Commands.runOnce(() -> m_extendArm.setControllerAtPosition(),
                                                                m_extendArm)));

                m_coDriverController.rightBumper().whileTrue(getJogWristCommand(m_coDriverController))

                                .onFalse(Commands.waitUntil(() -> m_wrist.isStopped())

                                                .andThen(Commands.runOnce(() -> m_wrist.setControllerAtPosition(),
                                                                m_wrist)));

                m_coDriverController.a().onTrue(getGroundIntake(gamePiece.CONE)
                                .withTimeout(2));

                m_coDriverController.b().onTrue(getGroundIntake(gamePiece.CUBE)
                                .withTimeout(2));

                m_coDriverController.x().onTrue((new GroundIntakeTippedConePositions(m_liftArm, m_wrist,
                                m_extendArm, m_intake)
                                .withName("Ground Tipped Cone Pickup Positions")
                                .withTimeout(2)));

                m_coDriverController.y().onTrue(deliverPositionsCommand(true).withTimeout(2));

                m_coDriverController.start().onTrue(deliverPositionsCommand(false).withTimeout(2));

                m_coDriverController.povUp().whileTrue(getJogLiftArmFixedCommand(-1, m_coDriverController))

                .onFalse(Commands.waitUntil(() -> m_liftArm.isStopped())

                                .andThen(Commands.runOnce(() -> m_liftArm.setControllerAtPosition(),
                                                m_liftArm)));

                m_coDriverController.povDown().whileTrue(getJogLiftArmFixedCommand(1, m_coDriverController))

                                .onFalse(Commands.waitUntil(() -> m_liftArm.isStopped())

                                                .andThen(Commands.runOnce(() -> m_liftArm.setControllerAtPosition(),
                                                                m_liftArm)));
                

                m_coDriverController.povLeft().whileTrue(new JogExtendArm(m_extendArm, () -> -.4, m_driverController))
                                .onFalse(Commands.waitUntil(() -> m_extendArm.isStopped())
                                                .andThen(Commands.runOnce(() -> m_extendArm.setControllerAtPosition(),
                                                                m_extendArm)));

                m_coDriverController.povRight().whileTrue(new JogExtendArm(m_extendArm, () -> -.4, m_driverController))
                                .onFalse(Commands.waitUntil(() -> m_extendArm.isStopped())
                                                .andThen(Commands.runOnce(() -> m_extendArm.setControllerAtPosition(),
                                                                m_extendArm)));


                m_coDriverController.rightTrigger().onTrue(

                                new InstantCommand(() -> m_tf.createSelectedTrajectory(2, 2, true)))

                                .onFalse(new InstantCommand(() -> m_tf.doSelectedTrajectory()));

                // m_coDriverController.back() DO NOT ASSIGN ALREADY USED IN JOG COMMANDS TO
                // OVERRIDE SOFTWARE LIMITS

        }

        public Command getDriveCommand() {
                return new SetSwerveDrive(m_drive,
                                () -> m_driverController.getRawAxis(1),
                                () -> m_driverController.getRawAxis(0),
                                () -> m_driverController.getRawAxis(4));

        }

        public Command getJogLiftArmCommand(CommandXboxController m_armController2) {

                return new JogLiftArm(m_liftArm, () -> -m_coDriverController.getRawAxis(1), m_armController2);
        }

        public Command getJogLiftArmFixedCommand(double speed, CommandXboxController m_armController2) {

                return new JogLiftArm(m_liftArm, () -> -speed, m_armController2);
        }

        public Command getJogWristCommand(CommandXboxController m_armController2) {

                return new JogWrist(m_wrist, () -> m_coDriverController.getRawAxis(5), m_armController2);
        }

        public Command getMoveWristCommand(double speed) {

                return new JogWrist(m_wrist, () -> speed, m_driverController);
        }

        public Command getStopDriveCommand() {
                return new InstantCommand(() -> m_drive.stopModules());
        }

        public Command getJogExtendArmCommand(CommandXboxController m_armController2) {
                return new JogExtendArm(m_extendArm, () -> -m_coDriverController.getRawAxis(0), m_armController2);

        }

        public Command getJogIntakeCommand() {
                return new JogIntake(m_intake, () -> m_coDriverController.getRawAxis(4));
        }

        public Command deliverPositionsCommand(boolean toplevel) {

                return new GetDeliverAngleSettings(m_liftArm, m_extendArm, m_wrist, m_intake,
                                toplevel);

        }

        public Command getGroundIntake(gamePiece type) {
                return new GroundIntakePositions(m_liftArm, m_wrist, m_extendArm, m_intake, type);

        }

        public Command getDetectorLoad() {
                return new DetectorLoad(m_drive, m_llv, m_ghs, () -> m_driverController.getRawAxis(1))
                                .andThen(new RumbleCommand(m_driverController, RumbleType.kLeftRumble, .5, 2));
        }

        public Command getLoadSettings() {

                return new LiftWristPresetLoadStation(m_liftArm, m_wrist, m_intake, m_ghs);

        }

        public Command raiseLowerWrist(boolean direction) {
                return new RaiseLowerWrist(m_wrist, direction);
        }

        public Command redoGoals() {

                return new ParallelCommandGroup(
                                new InstantCommand(() -> m_liftArm.redoTarget()),
                                new InstantCommand(() -> m_wrist.redoTarget()),
                                new InstantCommand(() -> m_extendArm.redoTarget()));
        }

        public void simulationPeriodic() {

                m_fieldSim.periodic();
        }

        public void periodic() {
                m_fieldSim.periodic();
                // m_pt.update();

        }
}
