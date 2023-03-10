package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Pref;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.ExtendArmConstants;

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

        RETRACT(0),

        HOME(1),

        SAFE_TRAVEL(2),

        PICKUP_CUBE_GROUND(15.7),

        PICKUP_CONE_GROUND(15.7),

        PICKUP_TIPPED_CONE_GROUND(118.2),

        PLACE_CUBE_MID_SHELF(4.9),

        PLACE_CUBE_TOP_SHELF(22.7),

        PICKUP_CONE_LOAD_STATION(1.),

        PICKUP_CUBE_LOAD_STATION(4.2),

        PLACE_CONE_MID_PIPE(5.3),

        PLACE_CONE_TOP_PIPE(26);

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

    public ProfiledPIDController m_extController = new ProfiledPIDController(0.005, 0, 0,
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

    public double inchespersec;

    private int loopctr;

    public int tstctr;

    public double ff;// feedforward

    public boolean resetFF;

    public double commandIPS;

    public double goalInches;

    public boolean firstUp;

    public boolean endComm;

    public double pidVal;

    public double volts;

    public boolean inIZone;

    private boolean runDeliverAngle;

    public ExtendArmSubsystem() {

        firstUp = true;

        useSoftwareLimit = false;

        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(true);
        m_motor.setOpenLoopRampRate(.1);
        m_motor.setClosedLoopRampRate(.1);

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);// vel

        mEncoder.setPositionConversionFactor(ExtendArmConstants.INCHES_PER_ENCODER_REV);

        mEncoder.setVelocityConversionFactor(ExtendArmConstants.INCHES_PER_ENCODER_REV / 60);

        mEncoder.setPosition(presetExtArmDistances.HOME.getDistance());

        m_motor.setSmartCurrentLimit(25);

        m_motor.setIdleMode(IdleMode.kBrake);

        mEncoder.setPosition(presetExtArmDistances.HOME.getDistance());

        setController(ExtendArmConstants.extendArmConstraints, presetExtArmDistances.HOME.getDistance(), true);

        if (RobotBase.isSimulation()) {

            setControllerGoal(ExtendArmConstants.MIN_POSITION);

        }

        setSoftwareLimits();

        enableSoftLimits(useSoftwareLimit);

        m_feedforward = new SimpleMotorFeedforward(ExtendArmConstants.ksExtArmVolts,
                ExtendArmConstants.kvExtArmVoltSecondsPerInch);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        if (runDeliverAngle) {

            setController(ExtendArmConstants.deliverConstraints, deliverDistance, false);

            runDeliverAngle = false;
        }

        loopctr++;

        if (faultSeen != 0)

            faultSeen = getFaults();

        // for Shuffleboard
        if (loopctr >= 5) {
            appliedOutput = getAppliedOutput();
            amps = getAmps();
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

    public void setControllerGoal(double position) {
        m_extController.setGoal(position);
    }

    public void setController(TrapezoidProfile.Constraints constraints, double distance, boolean initial) {

        if (isStopped()) {

            setControllerConstraints(constraints);
            setControllerGoal(distance);
            goalInches = distance;
            if (initial)
                m_extController.reset(new TrapezoidProfile.State(presetExtArmDistances.HOME.getDistance(), 0));
            else
                m_extController.reset(new TrapezoidProfile.State(getPositionInches(), 0));

        }
        m_feedforward = new SimpleMotorFeedforward(Pref.getPref("extKs"), Pref.getPref("extKv"));

        m_extController.setP(Pref.getPref("extKp"));
    }

    public void setControllerAtPosition() {
        setController(ExtendArmConstants.extendArmConstraints, getPositionInches(), false);

    }

    public void runDeliverAngle(boolean on) {

        runDeliverAngle = on;

    }

    public void resetPosition(double position) {

        mEncoder.setPosition(position);

    }

    public boolean atTargetPosition() {
        return Math.abs(goalInches - getPositionInches()) < inPositionBandwidth;
    }

    public boolean isStopped() {
        return Math.abs(mEncoder.getVelocity()) < .5;
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
