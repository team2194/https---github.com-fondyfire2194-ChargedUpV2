// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.SimConstants;
import frc.robot.subsystems.ExtendArmSubsystem.presetExtArmDistances;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;

  private double m_disableStartTime;

  private boolean driveIsBraked;

  public static int lpctra;

  private int loopCtr;

  private boolean lastOKState;

  private boolean firstScan = true;

  private boolean autoHasRun;
  private double m_startDelay;
  private double startTime;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    if (RobotBase.isReal())
      DataLogManager.start("/media/sda1");
    // Instantiate our RobotContainer.
    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // m_robotContainer.m_tf.periodic();

    SmartDashboard.putNumber("LP", lpctra++);

    // m_loop.poll();
    if (RobotBase.isSimulation())
      m_robotContainer.m_fieldSim.periodic();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_disableStartTime = 0;
    firstScan = true;
    // CommandScheduler.getInstance().cancelAll();
    // Shuffleboard.selectTab("Competition");
  }

  @Override
  public void disabledPeriodic() {

    loopCtr++;

    // if (loopCtr < 5) {

    // m_robotContainer.m_ghs.CANOK = m_robotContainer.m_drive.checkCANOK() &&
    // m_robotContainer.m_liftArm.checkCANOK()
    // && m_robotContainer.m_extendArm.checkCANOK() &&
    // m_robotContainer.m_wrist.checkCANOK()
    // && m_robotContainer.m_intake.checkCANOK() &&
    // m_robotContainer.m_intake.pieceSensorsOK;

    // loopCtr = 0;
    // }

    if (!m_robotContainer.m_drive.isbraked()) {
      if (m_disableStartTime == 0)
        m_disableStartTime = Timer.getFPGATimestamp();

      if (m_disableStartTime != 0 && Timer.getFPGATimestamp() > m_disableStartTime + 3) {
        m_robotContainer.m_drive.setIdleMode(false);
      }
    }

    // if (firstScan || m_robotContainer.m_ghs.CANOK != lastOKState) {
    // if (m_robotContainer.m_ghs.CANOK)
    // LightStrip.setColor(ledColors.GREEN);
    // else
    // LightStrip.setColor(ledColors.RED);
    // lastOKState = m_robotContainer.m_ghs.CANOK;
    // firstScan = false;
    // }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    autoHasRun = false;

    m_robotContainer.m_drive.setIdleMode(true);

    m_robotContainer.m_extendArm.setController(ExtendArmConstants.extendArmFastConstraints,
        presetExtArmDistances.RETRACT.getDistance(), false);

    m_robotContainer.m_liftArm.setController(LiftArmConstants.liftArmFastConstraints,
        m_robotContainer.m_liftArm.getPositionInches(), false);

    m_robotContainer.m_wrist.setController(WristConstants.wristFastConstraints,
        m_robotContainer.m_wrist.getAngleRadians(), false);

    m_startDelay = m_robotContainer.m_autoFactory.m_startDelayChooser.getSelected();

    startTime = Timer.getFPGATimestamp();

    m_robotContainer.m_autoFactory.createCommands();

    m_robotContainer.m_autoFactory.autonomousCommand = m_robotContainer.m_autoFactory.getAutonomousCommand();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (RobotBase.isSimulation())
      m_robotContainer.m_fieldSim.periodic();

    if (!autoHasRun && Timer.getFPGATimestamp() > startTime + m_startDelay
        && m_robotContainer.m_autoFactory.autonomousCommand != null) {
      m_robotContainer.m_autoFactory.autonomousCommand.schedule();
      autoHasRun = true;
    }

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the a
    // continue until interrupted by another command, remove
    // this line or comment it out.
    autoHasRun = false;

    m_robotContainer.m_drive.resetGyro();
    m_robotContainer.m_extendArm.setController(ExtendArmConstants.extendArmFastConstraints,
        presetExtArmDistances.RETRACT.getDistance(), false);

    m_robotContainer.m_drive.setIdleMode(true);

    if (m_robotContainer.m_autoFactory.autonomousCommand != null) {
      m_robotContainer.m_autoFactory.autonomousCommand.cancel();

    }
    // m_robotContainer.m_extendArm.setController(ExtendArmConstants.extendArmFastConstraints,
    // m_robotContainer.m_extendArm.getPositionInches(), false);

    m_robotContainer.m_liftArm.setController(LiftArmConstants.liftArmFastConstraints,
        m_robotContainer.m_liftArm.getPositionInches(), false);

    m_robotContainer.m_wrist.setController(WristConstants.wristFastConstraints,
        m_robotContainer.m_wrist.getAngleRadians(), false);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (RobotBase.isSimulation())
      m_robotContainer.m_fieldSim.periodic();

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    m_robotContainer.m_drive.setIdleMode(true);

    CommandScheduler.getInstance().cancelAll();

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }

  @Override
  public void simulationPeriodic() {
    m_robotContainer.m_fieldSim.periodic();
    // m_robotContainer.simulationPeriodic();
  }

  private void setAllianceStartPose() {
    Pose2d startPoseBlue = m_robotContainer.m_ghs.blueRightHybridNode.plus(new Transform2d(
        new Translation2d(SimConstants.robotLengthMeters / 2, SimConstants.robotWidthMeters),
        new Rotation2d()));
    // SmartDashboard.putString(("BlueStart"), startPoseBlue.toString());
    if (DriverStation.getAlliance() == Alliance.Blue) {
      new SetSwerveOdometry(m_robotContainer.m_drive,
          m_robotContainer.m_fieldSim, startPoseBlue)
          .schedule();
    }

    Pose2d startPoseRed = m_robotContainer.m_ghs.redLeftHybridNode.plus(new Transform2d(
        new Translation2d(SimConstants.robotLengthMeters / 2, SimConstants.robotWidthMeters),
        new Rotation2d()));
    if (DriverStation.getAlliance() == Alliance.Red) {
      new SetSwerveOdometry(m_robotContainer.m_drive,
          m_robotContainer.m_fieldSim, startPoseRed)
          .schedule();
    }

  }

}
