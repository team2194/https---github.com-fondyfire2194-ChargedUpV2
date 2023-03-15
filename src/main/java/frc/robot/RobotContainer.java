
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DeliverRoutines.DeliverSelectedPieceToSelectedTarget;
import frc.robot.commands.DeliverRoutines.EjectPieceFromIntake;
import frc.robot.commands.DeliverRoutines.GetDeliverAngleSettings;
import frc.robot.commands.DeliverRoutines.SetSwerveDriveTape;
import frc.robot.commands.ExtendArm.JogExtendArm;
import frc.robot.commands.ExtendArm.PositionProfileExtendArm;
import frc.robot.commands.Intake.JogIntake;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.LiftArm.JogLiftArm;
import frc.robot.commands.LiftArm.PositionProfileLiftInches;
import frc.robot.commands.NTs.MonitorThreadExt;
import frc.robot.commands.NTs.MonitorThreadIntake;
import frc.robot.commands.NTs.MonitorThreadLift;
import frc.robot.commands.NTs.MonitorThreadWrist;
import frc.robot.commands.PickupRoutines.DetectorLoad;
import frc.robot.commands.PickupRoutines.GetPieceAtIntake;
import frc.robot.commands.PickupRoutines.GroundIntake;
import frc.robot.commands.PickupRoutines.GroundIntakeTippedCone;
import frc.robot.commands.TeleopRoutines.RetractWristExtendLift;
import frc.robot.commands.TeleopRoutines.RotateToAngle;
import frc.robot.commands.TeleopRoutines.StrafeToGridSlot;
import frc.robot.commands.Wrist.JogWrist;
import frc.robot.commands.Wrist.PositionProfileWrist;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.oi.ShuffleboardArms;
import frc.robot.oi.ShuffleboardCompetition;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem.presetExtArmDistances;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem.gamePiece;
import frc.robot.subsystems.GameHandlerSubsystem.robotPiece;
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

        public CommandXboxController m_armController = new CommandXboxController(
                        OIConstants.kArmControllerPort);

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

                mlift.startThread();

                m_extendArm = new ExtendArmSubsystem();

                mext = new MonitorThreadExt(m_extendArm);

                mext.startThread();

                mwrist = new MonitorThreadWrist(m_wrist);

                mwrist.startThread();

                mIntake = new MonitorThreadIntake(m_intake);

                mIntake.startThread();

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

                configArmControllerButtons();

        }

        private void setDefaultCommands() {

                m_drive.setDefaultCommand(getDriveCommand());

                m_extendArm.setDefaultCommand(new PositionProfileExtendArm(m_extendArm,
                                m_liftArm));

                // m_liftArm.setDefaultCommand(
                // new PositionProfileLift(m_liftArm));

                m_liftArm.setDefaultCommand(new PositionProfileLiftInches(m_liftArm));

                m_wrist.setDefaultCommand(
                                new PositionProfileWrist(m_wrist, m_liftArm));

        }

        void configDriverButtons() {

                m_driverController.leftBumper()
                                .whileTrue(getDetectorLoad());

                m_driverController.rightBumper()
                                .onTrue(new RetractWristExtendLift(m_liftArm, m_extendArm, m_wrist, true));

                m_driverController.leftTrigger().whileTrue((new StrafeToGridSlot(m_drive, m_tf,
                                m_ghs, m_llv, m_intake)));

                m_driverController.rightTrigger().whileTrue(new SetSwerveDriveTape(m_drive, m_llv, m_ghs,
                                () -> m_driverController.getRawAxis(1)));

                m_driverController.x()
                                .onTrue(new GroundIntake(m_liftArm, m_wrist, m_extendArm, m_intake, gamePiece.CONE)
                                                .withName("Ground  Cone Pickup")
                                                .withTimeout(1));

                m_driverController.y()
                                .onTrue(new GroundIntake(m_liftArm, m_wrist, m_extendArm, m_intake, gamePiece.CUBE)
                                                .withName("Ground  Cube Pickup")
                                                .withTimeout(1));

                m_driverController.a().onTrue(getDeliverCommand());

                m_driverController.a().onTrue(new GroundIntakeTippedCone(m_liftArm, m_wrist, m_extendArm, m_intake)
                                .withName("Ground Tipped Cone Pickup")
                                .withTimeout(1).withTimeout(2));

                m_driverController.povLeft()
                                .onTrue(new SequentialCommandGroup(
                                                Commands.runOnce(() -> m_drive.setInhibitVisionCorrection(true)),
                                                new RotateToAngle(m_drive, 0).withTimeout(300),
                                                Commands.runOnce(() -> m_drive.setInhibitVisionCorrection(false)),
                                                Commands.runOnce(() -> m_drive.setIsRotating(true)))
                                                .withName("RotateToZero").andThen(() -> m_drive.stopModules()));
                m_driverController.povRight()
                                .onTrue(new SequentialCommandGroup(
                                                Commands.runOnce(() -> m_drive.setInhibitVisionCorrection(true)),
                                                new RotateToAngle(m_drive, 180).withTimeout(300).withName("LIFT"),
                                                Commands.runOnce(() -> m_drive.setInhibitVisionCorrection(false)),
                                                Commands.runOnce(() -> m_drive.setIsRotating(true)))
                                                .withName("RotateTo180").andThen(() -> m_drive.stopModules()));

                m_driverController.povDown().onTrue(Commands.runOnce((() -> m_liftArm.redoTarget())))
                                .onTrue(Commands.runOnce((() -> m_wrist.redoTarget())))
                                .onTrue(Commands.runOnce((() -> m_extendArm.redoTarget())));

                // m_driverController.povLeft()

                // m_driverController.start()

                // m_driverController.back()
        }

        private void configCodriverButtons() {

                m_coDriverController.leftBumper()
                                .onTrue(new DeliverSelectedPieceToSelectedTarget(m_liftArm, m_extendArm, m_wrist,
                                                m_intake, m_ghs).withTimeout(2));

                // m_coDriverController.rightBumper()

                // m_coDriverController.leftTrigger()

                // m_coDriverController.rightTrigger()

                m_coDriverController.a().onTrue(new EjectPieceFromIntake(m_intake));

                // m_coDriverController.b()

                m_coDriverController.x().onTrue(new GetPieceAtIntake(m_intake, gamePiece.CONE));

                m_coDriverController.y().onTrue(new GetPieceAtIntake(m_intake, gamePiece.CUBE));

                m_coDriverController.povRight().onTrue(Commands.runOnce(() -> m_ghs.incDropNumber()));

                m_coDriverController.povLeft().onTrue(Commands.runOnce(() -> m_ghs.decDropNumber()));

                m_coDriverController.povUp().onTrue(Commands.runOnce(() -> m_ghs.stepDropOffLevel()));

                m_coDriverController.povDown().onTrue(Commands.runOnce(() -> m_ghs.toggleGamePieceType()));

                // m_coDriverController.start()

                // m_coDriverController.back()

        }

        public void configArmControllerButtons() {

                m_armController.leftBumper().whileTrue(getJogLiftArmCommand(m_armController))

                                .onFalse(Commands.waitUntil(() -> m_liftArm.isStopped())

                                                .andThen(Commands.runOnce(() -> m_liftArm.setControllerAtPosition(),
                                                                m_liftArm)));

                m_armController.leftTrigger().whileTrue(getJogExtendArmCommand(m_armController))

                                .onFalse(Commands.waitUntil(() -> m_extendArm.isStopped())

                                                .andThen(Commands.runOnce(() -> m_extendArm.setControllerAtPosition(),
                                                                m_extendArm)));

                m_armController.rightBumper().whileTrue(getJogWristCommand(m_armController))

                                .onFalse(Commands.waitUntil(() -> m_wrist.isStopped())

                                                .andThen(Commands.runOnce(() -> m_wrist.setControllerAtPosition(),
                                                                m_wrist)));
                // get extend arm off the hook
                m_armController.rightTrigger().onTrue(Commands.runOnce(
                                () -> m_extendArm.setController(ExtendArmConstants.extendArmConstraints,
                                                presetExtArmDistances.RETRACT.getDistance(), false)));

                // m_armController.start()

                m_armController.a().onTrue(new GroundIntake(m_liftArm, m_wrist, m_extendArm, m_intake, gamePiece.CONE));

                m_armController.b().onTrue(new GroundIntake(m_liftArm, m_wrist, m_extendArm, m_intake, gamePiece.CUBE));

                m_armController.y().onTrue(new GroundIntakeTippedCone(m_liftArm, m_wrist, m_extendArm, m_intake));

                m_armController.x().onTrue(
                                Commands.runOnce(() -> m_liftArm.setController(LiftArmConstants.liftArmInchConstraints,
                                                0, false)));

                m_armController.povUp()
                                .onTrue(new GetDeliverAngleSettings(m_liftArm, m_extendArm, m_wrist, m_intake, m_ghs));

                m_armController.povRight()
                                .onTrue(Commands.runOnce(() -> m_liftArm.runDeliverPosition()).withTimeout(5));

                m_armController.povDown().onTrue(Commands.runOnce(() -> m_wrist.runDeliverAngle(true)));

                m_armController.povLeft().onTrue(Commands.runOnce(() -> m_extendArm.runDeliverAngle(true)));
        }

        public Command getDriveCommand() {
                return new SetSwerveDrive(m_drive,
                                () -> m_driverController.getRawAxis(1),
                                () -> m_driverController.getRawAxis(0),
                                () -> m_driverController.getRawAxis(4));

        }

        public Command getJogLiftArmCommand(CommandXboxController m_armController2) {

                return new JogLiftArm(m_liftArm, () -> -m_armController.getRawAxis(1), m_armController2);
        }

        public Command getJogWristCommand(CommandXboxController m_armController2) {

                return new JogWrist(m_wrist, () -> m_armController.getRawAxis(5), m_armController2);
        }

        public Command getStopDriveCommand() {
                return new InstantCommand(() -> m_drive.stopModules());
        }

        public Command getJogExtendArmCommand(CommandXboxController m_armController2) {
                return new JogExtendArm(m_extendArm, () -> -m_armController.getRawAxis(0), m_armController2);

        }

        public Command getJogIntakeCommand() {
                return new JogIntake(m_intake, () -> m_armController.getRawAxis(4));
        }

        public Command getDeliverCommand() {
                return new DeliverSelectedPieceToSelectedTarget(m_liftArm, m_extendArm, m_wrist, m_intake, m_ghs)
                                .withTimeout(5);
        }

        public Command getDetectorLoad() {
                return new DetectorLoad(m_drive, m_llv, m_ghs, null, false);
        }

        public void simulationPeriodic() {

                m_fieldSim.periodic();
        }

        public void periodic() {
                m_fieldSim.periodic();
                // m_pt.update();

        }
}
