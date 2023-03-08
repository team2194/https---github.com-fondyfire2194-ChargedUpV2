package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem.fieldTagsBlue;
import frc.robot.subsystems.GameHandlerSubsystem.fieldTagsRed;
import frc.robot.subsystems.LimelightVision;
import frc.robot.utils.TrajectoryFactory;

public class GetRightLoadValues extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_drive;
  private final LimelightVision m_llv;
  private final TrajectoryFactory m_tf;


  public fieldTagsRed activeTagRed;

  public fieldTagsBlue activeTagBlue;

  double xDistanceToTag;

  double yDistanceToTag;

  double hypotDistanceToTag;

  private Pose2d activeTagPose;

  /**
   * Creates a new Command.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public GetRightLoadValues(
      DriveSubsystem drive,
      LimelightVision llv,
      TrajectoryFactory tf) {

    m_drive = drive;
    m_llv = llv;
    m_tf = tf;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotBase.isReal())
      m_llv.setLoadPipeline();
  }

  @Override
  public void execute() {

    if (DriverStation.getAlliance() == Alliance.Red) {
      activeTagRed = fieldTagsRed.RED_LOAD;
      activeTagPose = activeTagRed.getPose();

    }

    else {
      activeTagBlue = fieldTagsBlue.BLUE_LOAD;
      activeTagPose = activeTagBlue.getPose();
    }

    m_drive.setActiveTagPose(activeTagPose);

    m_tf.setActiveTagPose(activeTagPose);

    m_tf.setLeftPickup(false);

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
