package frc.robot.commands.PickupRoutines;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem.fieldTagsBlue;
import frc.robot.subsystems.GameHandlerSubsystem.fieldTagsRed;
import frc.robot.subsystems.GameHandlerSubsystem.gamePiece;
import frc.robot.subsystems.LimelightVision;

public class DetectorLoad extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_drive;
  private final LimelightVision m_llv;
  private final GameHandlerSubsystem m_ghs;
  private final SlewRateLimiter m_slewX = new SlewRateLimiter(DriverConstants.kTranslationSlew);
  private final DoubleSupplier m_throttleInput;

  private double throttle;
  double offset = 0;
  double loadY = 0;
  // double yTolerance = .1;
  private double kpY = .35;
  private double kpR = .01;
  private double endX;
  private final double stopDistance = 1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public DetectorLoad(
      DriveSubsystem drive,
      LimelightVision llv,
      GameHandlerSubsystem ghs,
      DoubleSupplier throttleInput) {

    m_drive = drive;
    m_llv = llv;
    m_ghs = ghs;
    m_throttleInput = throttleInput;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (DriverStation.getAlliance() == Alliance.Blue) {

      loadY = fieldTagsBlue.BLUE_LOAD.getPose().getY();

      endX = fieldTagsBlue.BLUE_LOAD.getPose().getX();

    } else {

      loadY = fieldTagsRed.RED_LOAD.getPose().getY();

      endX = fieldTagsRed.RED_LOAD.getPose().getX();

    }
    if (m_ghs.gamePieceType == gamePiece.CONE) {
      
      m_llv.setConeDetectorPipeline();

    } else {
      m_llv.setCubeDetectorPipeline();
    }

  };

  /**
   * Limelight will detect the piee and provide tx angle
   * distance can be determined from April Tags so yhr Y coordinate of the piece
   * can be calculated
   */
  @Override
  public void execute() {
    throttle = MathUtil.applyDeadband(Math.abs(m_throttleInput.getAsDouble()),
        DriverConstants.kControllerDeadband)
        * Math.signum(m_throttleInput.getAsDouble());

    // square values after deadband while keeping original sign

    throttle *= -DriveConstants.kMaxSpeedMetersPerSecond / 3;

    double throttle_sl = m_slewX.calculate(throttle);

    double yError = m_drive.getY() - loadY;

    double rotError = -m_drive.tx * Pref.getPref("loadkPr");

    if (Math.abs(yError) > Pref.getPref(("loadYTol")))

      rotError = 0;

    yError *= Pref.getPref("loadkPY");

    if (m_drive.getX() > endX - Pref.getPref("loadStopDist")) {
      throttle_sl = 0;
      yError = 0;
      rotError = 0;

    }

    m_drive.drive(throttle_sl, -yError, rotError);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_drive.hasTag;
  }
}
