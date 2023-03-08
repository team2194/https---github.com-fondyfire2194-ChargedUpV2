package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.commands.ExtendArm.EndExtCommand;
import frc.robot.commands.ExtendArm.PositionProfileExtendArm;
import frc.robot.Pref;

public class ExtendArmSubsystem extends SubsystemBase {

    public enum presetExtArmDistances {

        /**
         * Exten ard distances ar in inches and range from 0 t0 25
         * Arm will be preset to 1 inch on startup. This allows for a short retract
         * to clear the wrist from the start up position
         * 
         * 
         * 
         */

        HOME(1),

        SAFE_TRAVEL(1),

        PICKUP_CUBE_GROUND_(28),

        PICKUP_CONE_GROUND_(27),

        PLACE_CUBE_GROUND(28.),

        PLACE_CUBE_MID_SHELF(5.),

        PLACE_CUBE_TOP_SHELF(8.),

        PICKUP_CONE_LOAD_STATION(1.),

        PICKUP_CUBE_LOAD_STATION(2.),

        PLACE_CONE_GROUND(28),

        PLACE_CONE_MID_SHELF(6.),

        PLACE_CONE_TOP_SHELF(9.),

        PLACE_CONE_MID_PIPE(7),

        PLACE_CONE_TOP_PIPE(24);

        private double distance;

        private presetExtArmDistances(double distance) {
            this.distance = distance;
        }

        public double getDistance() {
            return distance;
        }
    }

    public final int VELOCITY_SLOT = 0;

    public final CANSparkMax m_motor = new CANSparkMax(CanConstants.EXTEND_ARM_MOTOR,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder mEncoder = m_motor.getEncoder();
    public final SparkMaxPIDController mPosController;

    public ProfiledPIDController m_extController = new ProfiledPIDController(0.1, 0, 0,
            ExtendArmConstants.extendArmConstraints);

    private double inPositionBandwidth = .25;

    public boolean extendMotorConnected;

    public int faultSeen;

    private double m_positionSim;

    private double maxinchespersec = 2;

    public double inchespersecpervolt = maxinchespersec / 12;

    private double positionChangeper20ms;

    public SimpleMotorFeedforward m_feedforward;

    public double deliverDistance;

    private boolean useSoftwareLimit;

    public double appliedOutput;

    public double amps;

    public double positionInches;

    public double endpointInches;

    public double inchespersec;

    private int loopctr;

    private int tstctr;

    public double ff;// feedforward

    public boolean resetFF;

    public double commandIPS;

    public double goalInches;

    public boolean firstUp;

    private boolean runPos;

    public boolean endComm;

    public ExtendArmSubsystem() {

        firstUp = true;

        useSoftwareLimit = false;

        mPosController = m_motor.getPIDController();
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(true);
        m_motor.setOpenLoopRampRate(1);
        m_motor.setClosedLoopRampRate(1);

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);// vel

        mPosController.setOutputRange(-ExtendArmConstants.MAX_RATE_INCHES_PER_SEC,
                ExtendArmConstants.MAX_RATE_INCHES_PER_SEC);

        mPosController.setP(.1, 1);

        mEncoder.setPositionConversionFactor(ExtendArmConstants.INCHES_PER_ENCODER_REV);

        mEncoder.setVelocityConversionFactor(ExtendArmConstants.INCHES_PER_ENCODER_REV / 60);

        SmartDashboard.putNumber("EXTIPR", ExtendArmConstants.INCHES_PER_ENCODER_REV);

        mEncoder.setPosition(presetExtArmDistances.HOME.getDistance());

        setGoal(getPositionInches());

        m_motor.setSmartCurrentLimit(25);

        m_motor.setClosedLoopRampRate(2);

        m_motor.setIdleMode(IdleMode.kBrake);

        mEncoder.setPosition(0);

        if (RobotBase.isSimulation()) {

            setGoal(ExtendArmConstants.MIN_POSITION);

            mPosController.setP(5);
        }

        setSoftwareLimits();

        enableSoftLimits(useSoftwareLimit);

        m_feedforward = new SimpleMotorFeedforward(ExtendArmConstants.ksExtArmVolts,
                ExtendArmConstants.kvExtArmVoltSecondsPerInch);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        if (runPos) {
           // new EndExtCommand(this);
            new PositionProfileExtendArm(this, ExtendArmConstants.extendArmConstraints, Pref.getPref("EXTINCHES"))
                    .schedule();
            runPos = false;
        }

        SmartDashboard.putBoolean("RUNPOS", runPos);

        SmartDashboard.putBoolean("EXTSTOPPE", isStopped());

        loopctr++;

        if (RobotBase.isReal() && DriverStation.isDisabled())
            endpointInches = getPositionInches();

        if (faultSeen != 0)

            faultSeen = getFaults();

        // for Shuffleboard
        if (loopctr >= 5) {
            appliedOutput = getAppliedOutput();
            amps = getAmps();
            endpointInches = getEndpointInches();
            positionInches = getPositionInches();
            inchespersec = getInchesPerSec();
            extendMotorConnected = checkCANOK();
            loopctr = 0;
        }

    }

    @Override
    public void simulationPeriodic() {

        positionChangeper20ms = getAppliedOutput() * inchespersecpervolt / 50;

        m_positionSim += positionChangeper20ms;

    }

    public boolean checkCANOK() {
        return RobotBase.isSimulation() || m_motor.getFirmwareVersion() != 0;
    }

    public void close() {
        m_motor.close();
    }

    public void setControllerConstraints(TrapezoidProfile.Constraints constraints) {
        m_extController.setConstraints(constraints);
    }

    public void setGoal(double position) {
        goalInches = position;
    }

    public void resetPosition(double position) {

        mEncoder.setPosition(position);

        endpointInches = 0;

    }

    public boolean atTargetPosition() {
        return Math.abs(endpointInches - getPositionInches()) < inPositionBandwidth;
    }

    public boolean isStopped() {
        return Math.abs(mEncoder.getVelocity()) < .5;
    }

    public double getEndpointInches() {
        return endpointInches;
    }

    public double getPositionInches() {

        if (RobotBase.isReal())

            return mEncoder.getPosition();
        else
            return m_positionSim;
    }

    public double getAppliedOutput() {
        return m_motor.getAppliedOutput();
    }

    public double getAmps() {
        return m_motor.getOutputCurrent();
    }

    public double getInchesPerSec() {
        return mEncoder.getVelocity();
    }

    public String getFirmware() {
        return m_motor.getFirmwareString();
    }

    public boolean onPlusSoftwareLimit() {
        return m_motor.getFault(FaultID.kSoftLimitFwd);
    }

    public boolean onMinusSoftwareLimit() {
        return m_motor.getFault(FaultID.kSoftLimitRev);
    }

    public boolean onPlusHardwareLimit() {
        return m_motor.getFault(FaultID.kHardLimitRev);
    }

    public boolean onMinusHardwareLimit() {
        return m_motor.getFault(FaultID.kHardLimitRev);
    }

    public void stop() {

        m_motor.set(0);

    }

    public void setRunPos(boolean on) {
        runPos = on;
    }

    public Command startPosition() {
        runPos = false;
        return new PositionProfileExtendArm(this, ExtendArmConstants.extendArmConstraints, Pref.getPref("EXTINCHES"));

    }

    public void setSoftwareLimits() {
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) ExtendArmConstants.MIN_POSITION);
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) ExtendArmConstants.MAX_POSITION);
        m_motor.setIdleMode(IdleMode.kBrake);

    }

    public void enableSoftLimits(boolean on) {
        m_motor.enableSoftLimit(SoftLimitDirection.kForward, on);
        m_motor.enableSoftLimit(SoftLimitDirection.kReverse, on);
    }

    public boolean isBrake() {
        return m_motor.getIdleMode() == IdleMode.kBrake;
    }

    public boolean getSoftwareLimitsEnabled() {
        return m_motor.isSoftLimitEnabled(SoftLimitDirection.kForward)
                || m_motor.isSoftLimitEnabled(SoftLimitDirection.kReverse);
    }

    public void clearFaults() {
        m_motor.clearFaults();
        faultSeen = 0;
    }

    public int getFaults() {
        return m_motor.getFaults();
    }

}
