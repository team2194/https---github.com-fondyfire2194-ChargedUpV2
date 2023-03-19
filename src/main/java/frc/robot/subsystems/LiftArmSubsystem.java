package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.Pref;
import frc.robot.utils.AngleUtils;

public class LiftArmSubsystem extends SubsystemBase {

    /**
     * Lift arm angles are defined as 90 degrees horizontal falling to 34.2 base
     * The CANCoder is offset to meet that convention.
     * 
     */

    public enum presetLiftAngles {

        SAFE_HOME(34.1, 0),

        TRAVEL(36, .8),

        CLEAR_ARMS(38, .85),

        PICKUP_CUBE_GROUND(43.6, 2.7),

        PICKUP_CONE_GROUND(43.6, 2.7),

        PICKUP_TIPPED_CONE_GROUND(56, 4.8),

        PICKUP_CUBE_LOAD_STATION(84, 11.4),

        PICKUP_CONE_LOAD_STATION(89, 11.9),

        PLACE_CUBE_MID_SHELF(72, 8.8),

        PLACE_CUBE_TOP_SHELF(87, 12.5),

        PLACE_CONE_MID_PIPE(78.5, 8.9),

        PLACE_CONE_TOP_PIPE(95, 12.);

        private double angle;
        private double inches;

        private presetLiftAngles(double angle, double inches) {
            this.angle = angle;
            this.inches = inches;

        }

        public double getAngle() {
            return this.angle;
        }

        public double getInches() {
            return this.inches;
        }

    }

    public final int VELOCITY_SLOT = 0;

    public final CANSparkMax m_motor;

    private final RelativeEncoder mEncoder;

    public final CANCoder m_liftCANcoder;

    public ProfiledPIDController m_liftController = new ProfiledPIDController(0.25, 0, 0,
            LiftArmConstants.liftArmFastConstraints, .02);

    private boolean useSoftwareLimit;

    private double inPositionBandwidth = .25;

    private double inRangeBandwidth = 1;

    public boolean liftArmMotorConnected;

    public int faultSeen;

    public double positionError;

    private double m_positionSim;

    private double maxdegpersec = 50;

    public double degreespersecpervolt = maxdegpersec / 12;

    private double positionChangeper20ms;

    public int liftAngleSelect;

    public SimpleMotorFeedforward m_sff;

    public double deliverInches;

    public double appliedOutput;

    public double amps;

    public double cancoderPosition;

    public double positionrads;

    public double radianspersec;

    private int loopctr;

    public int tstCtr;

    public double ff;

    public boolean resetFF;

    public double goalInches;

    public double gravCalc;

    public double pidVal;

    public double volts;

    public boolean inIZone;

    public double deliverAngle;

    public double gravVal;

    public boolean atGoal;

    public double positioninches;

    public boolean stopped;

    public boolean topLevel;

    public LiftArmSubsystem() {
        useSoftwareLimit = true;
        m_motor = new CANSparkMax(CanConstants.LIFT_ARM_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(true);
        m_motor.setOpenLoopRampRate(.1);
        m_motor.setClosedLoopRampRate(.1);

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);// vel

        // absolute encoder used to establish known wheel position on start position
        m_liftCANcoder = new CANCoder(CanConstants.LIFT_CANCODER, "CV1");
        m_liftCANcoder.configFactoryDefault();
        m_liftCANcoder.configAllSettings(AngleUtils.generateCanCoderConfig());

        mEncoder.setPositionConversionFactor(LiftArmConstants.INCHES_PER_ENCODER_REV);
        // SmartDashboard.putNumber("LDPER", LiftArmConstants.DEGREES_PER_ENCODER_REV);
        mEncoder.setVelocityConversionFactor(LiftArmConstants.INCHES_PER_ENCODER_REV / 60);

        mEncoder.setPosition(0);

        setController(LiftArmConstants.liftArmFastConstraints, 0, false);

        m_motor.setSmartCurrentLimit(40);

        m_motor.setIdleMode(IdleMode.kBrake);

        if (RobotBase.isSimulation()) {

            m_positionSim = LiftArmConstants.MIN_INCHES;

        }

        setSoftwareLimits();

        enableSoftLimits(useSoftwareLimit);

        m_sff = new SimpleMotorFeedforward(LiftArmConstants.ksVolts,
                LiftArmConstants.kvVoltSecondsPerInch, LiftArmConstants.kAVoltSecondSquaredPerInch);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        loopctr++;

        if (faultSeen != 0)
            faultSeen = getFaults();

        if (loopctr == 25) {
          //  appliedOutput = getAppliedOutput();
            amps = getAmps();
            cancoderPosition = getCanCoderPosition();
          //  atGoal = m_liftController.atGoal();
           // liftArmMotorConnected = checkCANOK();
            stopped = isStopped();
        }
        if (loopctr == 36) {
            positioninches = getPositionInches();

            loopctr = 0;
        }

    }

    @Override
    public void simulationPeriodic() {

        positionChangeper20ms = getAppliedOutput() * degreespersecpervolt / 50;

        m_positionSim += positionChangeper20ms;

    }

    public boolean checkCANOK() {
        return RobotBase.isSimulation() || m_motor.getFirmwareVersion() != 0;
    }

    public void close() {
        m_motor.close();
    }

    public boolean atTargetPosition() {
        return Math.abs(goalInches - getPositionInches()) < inPositionBandwidth;
    }

    public boolean inRange() {
        return Math.abs(goalInches - getPositionInches()) < inRangeBandwidth;
    }

    // will be 0 at horizontal
    public double getCanCoderPosition() {
        return m_liftCANcoder.getAbsolutePosition() + LiftArmConstants.LIFT_CANCODER_OFFSET;
    }

    // will be 0 at horizontal
    public double getCanCoderRadians() {
        return Units.degreesToRadians(m_liftCANcoder.getAbsolutePosition())
                + Units.degreesToRadians(LiftArmConstants.LIFT_CANCODER_OFFSET);

    }

    public double getCanCoderRate() {
        return m_liftCANcoder.getVelocity();
    }

    public double getCanCoderRateRadsPerSec() {
        return Units.degreesToRadians(m_liftCANcoder.getVelocity());
    }

    public boolean isStopped() {
        return Math.abs(mEncoder.getVelocity()) < .05;
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
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) LiftArmConstants.MIN_INCHES);
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) LiftArmConstants.MAX_INCHES);
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

    public void setControllerGoal(double goalInches) {
        m_liftController.setGoal(goalInches);
    }

    public void setControllerConstraints(TrapezoidProfile.Constraints constraints) {
        m_liftController.setConstraints(constraints);
    }

    public void setController(TrapezoidProfile.Constraints constraints, double inches, boolean initial) {
        setControllerConstraints(constraints);
        setControllerGoal(inches);
        goalInches = inches;
        if (initial)
            m_liftController.reset(new TrapezoidProfile.State(0, 0));
        else
            m_liftController.reset(new TrapezoidProfile.State(getPositionInches(), getCanCoderRateRadsPerSec()));

        m_sff = new SimpleMotorFeedforward(Pref.getPref("liftKs"), Pref.getPref("liftKv"));

        m_liftController.setP(Pref.getPref("liftKp"));

    }

    public void setControllerAtPosition() {
        goalInches = getPositionInches();
        setController(LiftArmConstants.liftArmFastConstraints, goalInches, false);

    }

    public void redoTarget() {
        setController(LiftArmConstants.liftArmFastConstraints, goalInches, false);

    }

    public void incGoal(double val) {

        double temp = getPositionInches() + val;

        setController(LiftArmConstants.liftArmFastConstraints, temp, false);

    }

    public double round2dp(double number) {
        number = Math.round(number * 100);
        number /= 100;
        return number;
    }
}
