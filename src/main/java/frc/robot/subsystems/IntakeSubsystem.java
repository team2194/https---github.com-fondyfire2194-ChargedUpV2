package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Pref;
import frc.robot.Constants.CanConstants;
import frc.robot.subsystems.GameHandlerSubsystem.gamePiece;
import frc.robot.subsystems.GameHandlerSubsystem.robotPiece;

public class IntakeSubsystem extends SubsystemBase {

  public enum presetIntakeSpeeds {
    PICKUP_CONE(-.7),
    DELIVER_CONE(.9),
    CONE_SPEED_THREE(.8),
    PICKUP_CUBE(.7),
    DELIVER_CUBE(-.9),
    CUBE_SPEED_THREE(-1);

    private double speed;

    private presetIntakeSpeeds(double speed) {
      this.speed = speed;
    }

    public double getSpeed() {
      return this.speed;
    }

  }

  public final CANSparkMax mIntakeMotor; // NOPMD

  private final RelativeEncoder mEncoder;

  private SparkMaxPIDController mPidController;

  // Create instance of Time-Of_Flight driver for device 1
  private final TimeOfFlight m_intakeSensorCube = new TimeOfFlight(CanConstants.CUBE_SENSOR);

  private final TimeOfFlight m_intakeSensorCone = new TimeOfFlight(CanConstants.CONE_SENSOR);

  public final int VELOCITY_SLOT = 0;

  public boolean intakeMotorConnected;

  private double setRPM;

  public int cubeSensedDistance;

  public int coneSensedDistance;

  public boolean cubePresent;

  public boolean conePresent;

  public boolean coneSensorOK;

  public boolean cubeSensorOK;

  public boolean pieceSensorsOK;

  private int normalConeDistance;

  private int normalCubeDistance;

  private int loopctr;

  public double deliverSpeed;

  public int deliverSensor;

  public int tstctr;

  public robotPiece piece = robotPiece.CUBE;

  public gamePiece expectedPieceType = gamePiece.CUBE;

  public boolean coneExpected;

  public boolean cubeExpected;

  public double rpm;

  public double amps;

  public IntakeSubsystem() {

    mIntakeMotor = new CANSparkMax(CanConstants.INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    mEncoder = mIntakeMotor.getEncoder();

    mPidController = mIntakeMotor.getPIDController();

    mIntakeMotor.restoreFactoryDefaults();

    mPidController.setFF(1.8e-4, VELOCITY_SLOT);

    mPidController.setP(.0001, VELOCITY_SLOT);

    mIntakeMotor.setInverted(true);
    mIntakeMotor.setOpenLoopRampRate(1.);
    mIntakeMotor.setClosedLoopRampRate(.5);
    mIntakeMotor.enableVoltageCompensation(12);

    mIntakeMotor.setSmartCurrentLimit(20);

    mIntakeMotor.setIdleMode(IdleMode.kCoast);

    mEncoder.setPositionConversionFactor(1);

    mEncoder.setVelocityConversionFactor(1);

    if (RobotBase.isReal()) {
      // Configure time of flight sensor for short ranging mode and sample
      // distance every 40 ms
      m_intakeSensorCube.setRangingMode(RangingMode.Short, 40);

      m_intakeSensorCone.setRangingMode(RangingMode.Short, 40);

    }
  }

  @Override
  public void periodic() {

    loopctr++;

    if (loopctr == 15) {

      // intakeMotorConnected = checkCANOK();

      coneSensedDistance = getConeSensorDistance();

      conePresent = coneSensedDistance > 20 && coneSensedDistance < Pref.getPref("conedist");

      cubeSensedDistance = getCubeSensorDistance();

      cubePresent = cubeSensedDistance > 20 && cubeSensedDistance < Pref.getPref("cubedist");
    }

    if (loopctr == 18) {
      if (conePresent && !cubePresent)
        piece = robotPiece.CONE;

      if (cubePresent && !conePresent)
        piece = robotPiece.CUBE;

      if ((!conePresent && !cubePresent) || (conePresent && cubePresent))
        piece = robotPiece.NO_PIECE;

      rpm = getRPM();

    }
    if (loopctr == 34) {

      if (expectedPieceType == gamePiece.CONE) {
        coneExpected = true;
        cubeExpected = false;
      }
      if (expectedPieceType == gamePiece.CUBE) {
        coneExpected = false;
        cubeExpected = true;
      }

      amps = getAmps();

      pieceSensorsOK = true;// coneSensorOK && cubeSensorOK;

      loopctr = 0;

    }

  }

  @Override
  public void simulationPeriodic() {

  }

  public int getCubeSensorDistance() {
    if (RobotBase.isReal())
      return (int) m_intakeSensorCube.getRange();
    else
      return 0;
  }

  public int getConeSensorDistance() {
    if (RobotBase.isReal())
      return (int) m_intakeSensorCone.getRange();
    else
      return 0;
  }

  public int getCubeSensorOffset() {
    if (RobotBase.isReal())
      return getCubeSensorDistance() - normalCubeDistance;
    else
      return 0;
  }

  public int getConeSensorOffset() {
    if (RobotBase.isReal())
      return getConeSensorDistance() - normalConeDistance;
    else
      return 0;
  }

  public int getCubeSensorSigma() {
    if (RobotBase.isReal())
      return (int) m_intakeSensorCube.getRangeSigma();
    else
      return 0;
  }

  public int getConeSensorSigma() {
    if (RobotBase.isReal())
      return (int) m_intakeSensorCone.getRangeSigma();
    else
      return 0;
  }

  public String getCubeSensorStatus() {
    if (RobotBase.isReal())
      return m_intakeSensorCube.getStatus().toString();
    else
      return "Sim";
  }

  public String getConeSensorStatus() {
    if (RobotBase.isReal())
      return m_intakeSensorCone.getStatus().toString();
    else
      return "Sim";
  }

  public boolean getConeSensorOK() {
    if (RobotBase.isReal())
      return getConeSensorStatus() == "Valid";
    else
      return true;
  }

  public void setPieceType(gamePiece type) {
    expectedPieceType = type;
  }

  public void close() {
    mIntakeMotor.close();

  }

  public boolean selectSensor(gamePiece piece) {
    if (piece == gamePiece.CUBE)
      return conePresent;
    else
      return cubePresent;
  }

  // public void runIntakeAtVelocity(double rpm) {
  // setRPM = rpm;
  // mPidController.setReference(rpm, ControlType.kVelocity, VELOCITY_SLOT);

  // }

  public void moveManually(double speed) {
    // SmartDashboard.putNumber("INTSP", speed);
    mIntakeMotor.setVoltage(speed * 12);
  }

  public boolean checkCANOK() {
    return RobotBase.isSimulation() || mIntakeMotor.getFirmwareVersion() != 0;

  }

  public double getRPM() {
    return Math.round(mEncoder.getVelocity());
  }

  public boolean getIntakeAtSpeed() {
    return Math.abs(getRPM() - setRPM) < 25;

  }

  public boolean isStopped() {
    return Math.abs(getRPM()) < 10;
  }

  public String getFirmware() {
    return mIntakeMotor.getFirmwareString();
  }

  public double getAmps() {
    return mIntakeMotor.getOutputCurrent();
  }

  public void stop() {

    mIntakeMotor.stopMotor();

    mIntakeMotor.set(0);

  }

  public double getAppliedOutput() {
    return mIntakeMotor.getAppliedOutput();
  }

  public boolean gamepieceInIntake() {
    return true;
  }

  public void clearFaults() {
    mIntakeMotor.clearFaults();

  }

  public int getIntakeFaults() {
    return mIntakeMotor.getFaults();
  }

  public String faultAsBitString() {
    return Integer.toBinaryString(getIntakeFaults());
  }

  public void stopIntake() {
    Commands.run(() -> this.stop(), this);
  }

  private void setCANTimes(){
    
    mIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    mIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    mIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);

  }

}
