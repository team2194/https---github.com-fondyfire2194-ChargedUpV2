package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTRECanCoder;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.utils.AngleUtils;

public class LiftArmSubsystem extends SubsystemBase {

    /**
     * Lift arm angles are defined as 90 degrees horizontal falling to 34.2 base
     * The CANCoder is offset to meet that convention.
     * 
     */

    public enum presetLiftAngles {

        SAFE_HOME(34.1),

        TRAVEL(36),

        CLEAR_ARMS(38),

        PICKUP_CUBE_GROUND(45),

        PICKUP_CONE_GROUND(45),

        PICKUP_CUBE_LOAD_STATION(84),

        PICKUP_CONE_LOAD_STATION(82),

        PLACE_CUBE_GROUND(45),

        PLACE_CUBE_MID_SHELF(60),

        PLACE_CUBE_TOP_SHELF(72),

        PLACE_CONE_GROUND(45),

        PLACE_CONE_MID_SHELF(60.),

        PLACE_CONE_TOP_SHELF(81.),

        PLACE_CONE_MID_PIPE(75),

        PLACE_CONE_TOP_PIPE(88.3);

        private double angle;

        private presetLiftAngles(double angle) {
            this.angle = angle;

        }

        public double getAngleRads() {
            return Units.degreesToRadians(this.angle);
        }

    }

    public final int VELOCITY_SLOT = 0;

    public final CANSparkMax m_motor;

    private final RelativeEncoder mEncoder;

    public final SparkMaxPIDController mVelController;

    public final CTRECanCoder m_liftCANcoder;

    public ProfiledPIDController m_liftController = new ProfiledPIDController(.01, 0, 0,
            LiftArmConstants.liftArmConstraints, .02);

    private boolean useSoftwareLimit;

    private double inPositionBandwidth = .02;

    public boolean liftArmMotorConnected;

    public int faultSeen;

    public double positionError;

    private double m_positionSim;

    private double maxdegpersec = 50;

    public double degreespersecpervolt = maxdegpersec / 12;

    private double positionChangeper20ms;

    public int liftAngleSelect;

    public ArmFeedforward m_armFeedforward;

    public double deliverAngleRads;

    public double appliedOutput;

    public double amps;

    public double cancoderPosition;

    public double positionrads;

    public double radianspersec;

    private int loopctr;

    public int tstCtr;

    public double ff;

    public boolean resetFF;

    private double extEndAdjust;

    private double wristAngleAdj;

    public double goalAngleRadians;

    public double positionRadians;

    private boolean runPos;

    public double gravCalc;

    public boolean useVel;

    public LiftArmSubsystem() {
        useSoftwareLimit = true;
        m_motor = new CANSparkMax(CanConstants.LIFT_ARM_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        mVelController = m_motor.getPIDController();
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(true);
        m_motor.setOpenLoopRampRate(1);
        m_motor.setClosedLoopRampRate(1);

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);// vel

        // absolute encoder used to establish known wheel position on start position
        m_liftCANcoder = new CTRECanCoder(CanConstants.LIFT_CANCODER);
        m_liftCANcoder.configFactoryDefault();
        m_liftCANcoder.configAllSettings(AngleUtils.generateCanCoderConfig());

        mEncoder.setPositionConversionFactor(Units.degreesToRadians(LiftArmConstants.DEGREES_PER_ENCODER_REV));

        mEncoder.setVelocityConversionFactor(Units.degreesToRadians(LiftArmConstants.DEGREES_PER_ENCODER_REV / 60));

        // SmartDashboard.putNumber("LIIPR", LiftArmConstants.INCHES_PER_ENCODER_REV);

        mVelController.setOutputRange(-1, 1);

        mVelController.setFF(1 / LiftArmConstants.MAX_RAD_PER_SEC);

        m_motor.setSmartCurrentLimit(40);

        m_motor.setClosedLoopRampRate(.25);

        m_motor.setIdleMode(IdleMode.kBrake);

        if (RobotBase.isSimulation()) {

            m_positionSim = LiftArmConstants.MIN_INCHES;

        }

        setSoftwareLimits();

        enableSoftLimits(useSoftwareLimit);

        m_armFeedforward = new ArmFeedforward(LiftArmConstants.ksVolts, LiftArmConstants.kGVolts,
                LiftArmConstants.kvVoltSecondsPerRadian);

        setController(LiftArmConstants.liftArmConstraints,
                presetLiftAngles.SAFE_HOME.getAngleRads(), false);

        mEncoder.setPosition(presetLiftAngles.SAFE_HOME.getAngleRads());

    }

    // public double getInchesFromDegrees(double degrees) {

    // return LiftArmConstants.MIN_INCHES

    // + (degrees - LiftArmConstants.MIN_ANGLE) / LiftArmConstants.DEGREES_PER_INCH;
    // }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        loopctr++;

        if (faultSeen != 0)
            faultSeen = getFaults();

        if (loopctr == 5) {
            appliedOutput = getAppliedOutput();
            amps = getAmps();
            cancoderPosition = getCanCoderPosition();

        }
        if (loopctr == 6) {
            positionrads = getPositionRadians();
            radianspersec = getRadiansPerSec();
            liftArmMotorConnected = checkCANOK();
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

    public double getIaccum() {
        return mVelController.getIAccum();
    }

    public void close() {
        m_motor.close();
    }

    public void setExtAdjust(double adj) {

        extEndAdjust = adj;
    }

    public void setWristAdjust(double adj) {

        wristAngleAdj = adj;
    }

    public boolean atTargetAngle() {
        return Math.abs(goalAngleRadians - getCanCoderRadians()) < inPositionBandwidth;
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
        return m_liftCANcoder.getVelValue();
    }

    public double getCanCoderRateRadsPerSec() {

        return Units.degreesToRadians(m_liftCANcoder.getVelValue());
    }

    public boolean isStopped() {
        return Math.abs(mEncoder.getVelocity()) < .05;
    }

    public double getPositionRadians() {

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

    public double getRadiansPerSec() {
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

    public void setRunPos(boolean on) {
        runPos = on;
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

    public void setControllerGoal(double goalRadians) {
        goalAngleRadians = goalRadians;
    }

    public void setControllerConstraints(TrapezoidProfile.Constraints constraints) {
        m_liftController.setConstraints(constraints);
    }

    public void setController(TrapezoidProfile.Constraints constraints, double anglerads, boolean initial) {
        if (isStopped()) {
            setControllerConstraints(constraints);
            setControllerGoal(anglerads);
            if (initial)
                m_liftController.reset(new TrapezoidProfile.State(presetLiftAngles.SAFE_HOME.getAngleRads(), 0));
            else
                m_liftController.reset(new TrapezoidProfile.State(getCanCoderRadians(), 0));
        }
    }

    public void setControllerAtPosition() {
        setController(LiftArmConstants.liftArmConstraints, getCanCoderRadians(), false);

    }

}
