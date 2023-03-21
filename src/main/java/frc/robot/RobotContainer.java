
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
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.DeliverRoutines.DeliverPiecePositions;
import frc.robot.commands.DeliverRoutines.EjectPieceFromIntake;
import frc.robot.commands.DeliverRoutines.GetDeliverAngleSettings;
import frc.robot.commands.DeliverRoutines.SetSwerveDriveTape;
import frc.robot.commands.ExtendArm.JogExtendArm;
import frc.robot.commands.ExtendArm.PositionProfileExtendArm;
import frc.robot.commands.Intake.JogIntake;
import frc.robot.commands.LiftArm.JogLiftArm;
import frc.robot.commands.LiftArm.PositionProfileLiftInches;
import frc.robot.commands.NTs.MonitorThreadExt;
import frc.robot.commands.PickupRoutines.DetectorLoad;
import frc.robot.commands.PickupRoutines.GetPieceExpectedAtIntake;
import frc.robot.commands.PickupRoutines.GroundIntakePositions;
import frc.robot.commands.PickupRoutines.SetArmsForLoadPickup;
import frc.robot.commands.TeleopRoutines.RetractWristExtendLift;
import frc.robot.commands.Wrist.JogWrist;
import frc.robot.commands.Wrist.PositionProfileWrist;
import frc.robot.commands.Wrist.RaiseLowerWrist;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.commands.swerve.SetSwerveDriveSlow;
import frc.robot.oi.RumbleCommand;
import frc.robot.oi.ShuffleboardArms;
import frc.robot.oi.ShuffleboardCompetition;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem.gamePiece;
import frc.robot.subsystems.LightStrip.ledColors;
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

        final IntakeSubsystem m_intake;

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

        public CommandXboxController m_armsController = new CommandXboxController(
                        OIConstants.kArmControllerPort);

        // final PowerDistribution m_pdp = new PowerDistribution();

        public LimelightVision m_llvis = new LimelightVision();

        public LLDriveLinkerSubsystem m_lldv;

        public LightStrip m_ls = new LightStrip(9, 35);

        public MonitorThreadExt mext;
        // public MonitorThreadLift mlift;
        // public MonitorThreadWrist mwrist;
        // public MonitorThreadIntake mIntake;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                m_drive = new DriveSubsystem();

                Pref.deleteUnused();

                Pref.addMissing();

                m_llv = new LimelightVision();

                m_lldv = new LLDriveLinkerSubsystem(m_llv, m_drive);

                m_liftArm = new LiftArmSubsystem();

                // mlift = new MonitorThreadLift(m_liftArm);

                // mlift.startThread();

                // mext = new MonitorThreadExt(m_extendArm);

                // mext.startThread();

                // mwrist = new MonitorThreadWrist(m_wrist);

                // mwrist.startThread();

                // mIntake = new MonitorThreadIntake(m_intake);

                // mIntake.startThread();

                SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

                LiveWindow.disableAllTelemetry();

                m_intake = new IntakeSubsystem();

                m_ghs = new GameHandlerSubsystem();

                m_extendArm = new ExtendArmSubsystem();

                m_tf = new TrajectoryFactory(m_drive, m_fieldSim, m_ghs);

                m_autoFactory = new AutoFactory(m_drive, m_ghs, m_liftArm, m_extendArm, m_wrist, m_intake, m_tf);

                m_fieldSim = new FieldSim(m_drive, m_ghs);

                m_fieldSim.initSim();

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

                configArmsButtons();

        }

        private void setDefaultCommands() {

                m_drive.setDefaultCommand(getDriveCommand());

                m_extendArm.setDefaultCommand(new PositionProfileExtendArm(m_extendArm,
                                m_liftArm));

                m_liftArm.setDefaultCommand(new PositionProfileLiftInches(m_liftArm));

                m_wrist.setDefaultCommand(new PositionProfileWrist(m_wrist, m_liftArm));

        }

        void configDriverButtons() {

                m_driverController.leftTrigger()
                                .whileTrue(getDetectorLoad());

                m_driverController.leftBumper().whileTrue(new SetSwerveDriveTape(m_drive, m_llv, m_ghs,
                                () -> m_driverController.getRawAxis(1)));

                m_driverController.rightTrigger().whileTrue(getSlowDriveCommand());

                m_driverController.rightBumper().onTrue(deliverPositionsCommand(true).withTimeout(10));

                m_driverController.a().onTrue(new EjectPieceFromIntake(m_intake).withTimeout(1));

                m_driverController.b().onTrue(new GetPieceExpectedAtIntake(m_intake).withTimeout(5));

                m_driverController.x().onTrue(deliverPositionsCommand(false).withTimeout(10));

                m_driverController.y()
                                .onTrue(new SetArmsForLoadPickup(m_liftArm, m_wrist, m_extendArm, m_intake, m_ghs)
                                                .withTimeout(10));

                m_driverController.start().onTrue(new RetractWristExtendLift(m_liftArm, m_extendArm, m_wrist, true)
                                .withTimeout(8));

                m_driverController.back().whileTrue(Commands.run(m_drive::setX));

                m_driverController.povUp().onTrue(Commands.runOnce(() -> m_wrist.incGoal(.02)));

                m_driverController.povDown().onTrue(Commands.runOnce(() -> m_wrist.incGoal(-.02)));

                m_driverController.povLeft().onTrue(Commands.runOnce(() -> m_extendArm.incGoal(.25)));

                m_driverController.povRight().onTrue(Commands.runOnce(() -> m_extendArm.incGoal(-.25)));

        }

        private void configCodriverButtons() {

                m_coDriverController.leftBumper().onTrue(Commands.runOnce(() -> m_wrist.incGoal(.02)));

                m_coDriverController.rightBumper().onTrue(Commands.runOnce(() -> m_wrist.incGoal(-.02)));

                // m_coDriverController.leftTrigger().whileTrue(m_drive.autoBalance());

                m_coDriverController.a().onTrue(
                                new GroundIntakePositions(m_liftArm, m_wrist, m_extendArm, m_intake, gamePiece.CONE)
                                                .withTimeout(10));

                m_coDriverController.b().onTrue(
                                new GroundIntakePositions(m_liftArm, m_wrist, m_extendArm, m_intake, gamePiece.CUBE)
                                                .withTimeout(10));

                m_coDriverController.x().onTrue(Commands.runOnce(() -> m_ghs.toggleGamePieceType()))

                                .onTrue(Commands.runOnce(() -> m_ls.togglePY()));

                // m_coDriverController.y()

                // m_coDriverController.start().onTrue(deliverPositionsCommand(true).withTimeout(10));

                m_coDriverController.povUp().onTrue(Commands.runOnce(() -> m_liftArm.incGoal(.25)));

                m_coDriverController.povDown().onTrue(Commands.runOnce(() -> m_liftArm.incGoal(-.25)));

                m_coDriverController.povLeft().onTrue(Commands.runOnce(() -> m_extendArm.incGoal(.25)));

                m_coDriverController.povRight().onTrue(Commands.runOnce(() -> m_extendArm.incGoal(-.25)));

                m_coDriverController.rightTrigger().onTrue(Commands.runOnce(() -> m_ghs.toggleGamePieceType()));

        }

        private void configArmsButtons() {

                m_armsController.leftBumper().whileTrue(getJogLiftArmCommand(m_armsController))
                                .onFalse(Commands.runOnce(() -> m_liftArm.setControllerAtPosition(),
                                                m_liftArm));

                m_armsController.leftTrigger().whileTrue(getJogExtendArmCommand(m_armsController))
                                .onFalse(Commands.runOnce(() -> m_extendArm.setControllerAtPosition(),
                                                m_extendArm));

                m_armsController.rightBumper().whileTrue(getJogWristCommand(m_armsController))
                                .onFalse(Commands.waitUntil(() -> m_wrist.isStopped())
                                                .andThen(Commands.runOnce(() -> m_wrist.setControllerAtPosition(),
                                                                m_wrist)));

                m_armsController.a().onTrue(Commands.runOnce(
                                () -> m_wrist.setController(WristConstants.wristFastConstraints, 2, false)));

                m_armsController.b().onTrue(Commands.runOnce(
                                () -> m_wrist.setController(WristConstants.wristFastConstraints, 3, false)));

                m_armsController.x().onTrue(Commands.runOnce(
                                () -> m_liftArm.setController(LiftArmConstants.liftArmFastConstraints, 0, false)));

                m_armsController.y().onTrue(Commands.runOnce(
                                () -> m_liftArm.setController(LiftArmConstants.liftArmFastConstraints, 12, false)));

                // wrist

                m_armsController.start().onTrue(Commands.runOnce(
                                () -> m_wrist.setController(WristConstants.wristFastConstraints, 1, false)));

                m_armsController.povUp().onTrue(Commands.runOnce(
                                () -> m_wrist.setController(WristConstants.wristFastConstraints, 2.5, false)));

                // ext arm

                m_armsController.povDown().onTrue(Commands.runOnce(
                                () -> m_extendArm.setController(ExtendArmConstants.extendArmFastConstraints, 0,
                                                false)));

                m_armsController.povLeft().onTrue(Commands.runOnce(
                                () -> m_extendArm.setController(ExtendArmConstants.extendArmFastConstraints, 10,
                                                false)));

                m_armsController.povRight().onTrue(Commands.runOnce(
                                () -> m_extendArm.setController(ExtendArmConstants.extendArmFastConstraints, 15,
                                                false)));

                m_armsController.rightTrigger()
                                .onTrue(Commands.runOnce(() -> m_extendArm.setController(
                                                ExtendArmConstants.extendArmFastConstraints, 20,
                                                false)));

                // m_armsController.rightTrigger().onTrue(m_drive.autoBalance())
                // .onFalse(getStopDriveCommand());

                // m_armsController.back() DO NOT ASSIGN ALREADY USED IN JOG COMMANDS TO
                // OVERRIDE SOFTWARE LIMITS

        }

        public Command getDriveCommand() {
                return new SetSwerveDrive(m_drive,
                                () -> m_driverController.getRawAxis(1),
                                () -> m_driverController.getRawAxis(0),
                                () -> m_driverController.getRawAxis(4));

        }

        public Command getSlowDriveCommand() {
                return new SetSwerveDriveSlow(m_drive,
                                () -> m_driverController.getRawAxis(1),
                                () -> m_driverController.getRawAxis(0),
                                () -> m_driverController.getRawAxis(4));

        }

        public Command getJogLiftArmCommand(CommandXboxController m_armController2) {

                return new JogLiftArm(m_liftArm, () -> -m_armsController.getRawAxis(1), m_armController2);
        }

        public Command getJogWristCommand(CommandXboxController m_armController2) {

                return new JogWrist(m_wrist, () -> m_armsController.getRawAxis(5), m_armController2);
        }

        public Command getMoveWristCommand(double speed) {

                return new JogWrist(m_wrist, () -> speed, m_driverController);
        }

        public Command getStopDriveCommand() {
                return new InstantCommand(() -> m_drive.stopModules());
        }

        public Command getJogExtendArmCommand(CommandXboxController m_armController2) {
                return new JogExtendArm(m_extendArm, () -> -m_armsController.getRawAxis(0), m_armController2);

        }

        public Command getJogIntakeCommand() {
                return new JogIntake(m_intake, () -> m_coDriverController.getRawAxis(4));
        }

        public Command deliverPositionsCommand(boolean toplevel) {

                return new GetDeliverAngleSettings(m_liftArm, m_extendArm, m_wrist, m_intake,
                                toplevel);

        }

        public Command positionToDeliverCommand() {

                return new DeliverPiecePositions(m_liftArm, m_extendArm, m_wrist, m_intake);

        }

        public Command getPositionExtendCommand() {
                return Commands.runOnce(() -> m_extendArm.setController((ExtendArmConstants.extendArmFastConstraints),
                                m_extendArm.getNextTarget(), false));
        }

        public Command getMoveLiftCommand() {

                return Commands.runOnce(() -> m_liftArm.setController(LiftArmConstants.liftArmFastConstraints,
                                m_liftArm.deliverInches, false))
                                .andThen(Commands.runOnce(
                                                () -> m_wrist.setController(WristConstants.wristFastConstraints,
                                                                m_wrist.deliverAngleRads, false)));
        }

        public Command getMoveWristCommand() {

                return Commands.runOnce(() -> m_wrist.setController(WristConstants.wristFastConstraints,
                                m_wrist.deliverAngleRads, false));

        }

        public Command getDetectorLoad() {
                return new DetectorLoad(m_drive, m_llv, m_ghs, () -> m_driverController.getRawAxis(1))
                                .andThen(new RumbleCommand(m_driverController, RumbleType.kLeftRumble, .5, 2));
        }

        public Command getLoadSettings() {

                return new SetArmsForLoadPickup(m_liftArm, m_wrist, m_extendArm, m_intake, m_ghs);

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
