package frc.robot.commands.DeliverRoutines;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Pref;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem.dropOffLevel;
import frc.robot.subsystems.LimelightVision;

public class SetSwerveDriveTape extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_drive;
  private final LimelightVision m_llv;
  private final GameHandlerSubsystem m_ghs;
  private final SlewRateLimiter m_slewX = new SlewRateLimiter(DriverConstants.kTranslationSlew);
  private final DoubleSupplier m_throttleInput;

  private boolean lowLevel;

  private double throttle;
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
  public SetSwerveDriveTape(
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

    if (m_ghs.chosenLevel == dropOffLevel.MID_LEVEL) {

      m_llv.setLowTapePipeline();

      lowLevel = true;

    } else {

      m_llv.setHighTapePipeline();

      lowLevel = false;
    }

    loadY= m_ghs.getActiveDropPose().getY();
  };

  // https://www.chiefdelphi.com/t/swerve-controller-joystick/392544/5
  // Called every time the scheduler runs while the command is scheduled

  /**
   * Command is the line robot up to pickup either from the left or right sides of
   * the load station
   * Needs to adjust for red and blue alliances
   * Will get offset from Trajectory factory
   * 
   * Experimental
   *
   * Need to try out different combinations of strafe and rotate based on the tx
   * angle from LimeLight
   * 
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

    double temp = -5;
    if (!lowLevel) {
      temp = 15;
    }
    if (m_drive.ty < temp) {
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
    return !m_drive.hasTarget;
  }
}
