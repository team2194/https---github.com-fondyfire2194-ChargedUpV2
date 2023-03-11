package frc.robot.commands.DeliverRoutines;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.GameHandlerSubsystem.dropOffLevel;

/**
 * Command to position accurately in front of any grid slot.
 * Robot needs to synch its position with an April Tag before using this
 * Command will drive to given X value and strafe to Y
 * If slot is for a pipe then LimeLight tape pipeline will
 * be selected and used to stop the robot on center.
 * Driver controls strafe speed until PIC controller takes over.
 * 
 */

public class GetSlotValues extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_drive;
  private final GameHandlerSubsystem m_ghs;
  private final LimelightVision m_llv;
  private final IntakeSubsystem m_intake;

  public double intakeSensorOffset;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */

  public GetSlotValues(DriveSubsystem drive, GameHandlerSubsystem ghs, LimelightVision llv, IntakeSubsystem intake) {

    m_ghs = ghs;
    m_drive = drive;
    m_llv = llv;
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSensorOffset = 0;
    if (m_intake.conePresent)
      intakeSensorOffset = m_intake.getConeSensorOffset();
    if (m_intake.cubePresent)
      intakeSensorOffset = m_intake.getCubeSensorOffset();

    if (DriverStation.getAlliance() == Alliance.Blue) {

      m_drive.setEndY(m_ghs.getActiveDrop().getBluePose().getY() + intakeSensorOffset);
      m_drive.setEndX(m_ghs.getActiveDrop().getBluePose().getX() + m_ghs.strafeDistance);
      m_drive.isPipe = m_ghs.getActiveDrop().getIsPipeBlue();

    } else {
      m_drive.setEndY(m_ghs.getActiveDrop().getRedPose().getY() + intakeSensorOffset);
      m_drive.setEndX(m_ghs.getActiveDrop().getRedPose().getX() + m_ghs.strafeDistance);
      m_drive.isPipe = m_ghs.getActiveDrop().getIsPipeRed();
    }
    if (m_drive.isPipe && m_ghs.chosenLevel == dropOffLevel.MID_LEVEL)
      m_llv.setLowTapePipeline();
    if (m_drive.isPipe && m_ghs.chosenLevel == dropOffLevel.TOP_LEVEL)
      m_llv.setHighTapePipeline();
  }

  // https://www.chiefdelphi.com/t/swerve-controller-joystick/392544/5
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
