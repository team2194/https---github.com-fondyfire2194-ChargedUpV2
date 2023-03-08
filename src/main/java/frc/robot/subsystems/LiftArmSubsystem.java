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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTRECanCoder;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.commands.LiftArm.PositionProfileLift;
import frc.robot.Pref;
import frc.robot.utils.AngleUtils;

public class LiftArmSubsystem extends SubsystemBase {

    /**
     * Lift arm angles are defined as 90 degrees horizontal falling to 34.2 base
     * The CANCoder is offset to meet that convention.
     * 
     */

    public enum presetLiftAngles {

        SAFE_HOME(31.4),

        TRAVEL(33),

        CLEAR_ARMS(34),

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

        public double getAngle() {
            return Units.degreesToRadians(this.angle);
        }

    }

    public final int VELOCITY_SLOT = 0;

    public final CANSparkMax m_motor;

    private final RelativeEncoder mEncoder;

    public final SparkMaxPIDController mPosController;

    public final CTRECanCoder m_liftCANcoder;

    public ProfiledPIDController m_liftController = new ProfiledPIDController(.05, 0, 0,
            LiftArmConstants.liftArmConstraints, .02);

    private boolean useSoftwareLimit;

    private double inPositionBandwidth = .5;

    public boolean liftArmMotorConnected;

    public int faultSeen;

    public double positionError;

    public double endpointInches;

    private double pidout = 0;

    private double m_positionSim;

    private double maxdegpersec = 50;

    public double degreespersecpervolt = maxdegpersec / 12;

    private double positionChangeper20ms;

    public int liftAngleSelect;

    public ArmFeedforward m_armFeedforward;

    public double deliverAngle;

    public double appliedOutput;

    public double amps;

    public double cancoderPosition;

    public double positionInches;

    public double inchespersec;

    private int loopctr;

    public int tstCtr;

    public double ff;

    public boolean resetFF;

    private double extEndAdjust;

    private double wristAngleAdj;

    public double targetInchesFromSlider;
    public double commandIPS;

    public double goalAngleRadians;

    public double positionRadians;

    private boolean runPos;

    public LiftArmSubsystem() {
        useSoftwareLimit = false;
        m_motor = new CANSparkMax(CanConstants.LIFT_ARM_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        mPosController = m_motor.getPIDController();
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

        mPosController.setOutputRange(-LiftArmConstants.MAX_RATE_INCHES_PER_SEC,
                LiftArmConstants.MAX_RATE_INCHES_PER_SEC);

        mPosController.setP(.001);

        m_motor.setSmartCurrentLimit(40);

        m_motor.setClosedLoopRampRate(.5);

        m_motor.setIdleMode(IdleMode.kBrake);

        if (RobotBase.isSimulation()) {

            m_positionSim = LiftArmConstants.MIN_INCHES;

        }

        setSoftwareLimits();

        enableSoftLimits(useSoftwareLimit);

        m_armFeedforward = new ArmFeedforward(LiftArmConstants.kSVolts, LiftArmConstants.kGVolts,
                LiftArmConstants.kvVoltSecondsPerRadian);

    }

    public double getInchesFromDegrees(double degrees) {

        return LiftArmConstants.MIN_INCHES

                + (degrees - LiftArmConstants.MIN_ANGLE) / LiftArmConstants.DEGREES_PER_INCH;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        if (runPos) {
            // new EndExtCommand(this);

            double prefVAl = Pref.getPref("LIFTDEG");
            SmartDashboard.putNumber("PREFVAL", prefVAl);
            new PositionProfileLift(this, LiftArmConstants.liftArmConstraints,
            Pref.getPref("LIFTDEG"))
                    .schedule();
            runPos = false;
        }

        loopctr++;

        if (RobotBase.isReal() && DriverStation.isDisabled())
            endpointInches = getPositionInches();

        if (faultSeen != 0)
            faultSeen = getFaults();

        if (loopctr == 5) {
            appliedOutput = getAppliedOutput();
            amps = getAmps();
            cancoderPosition = getCanCoderPosition();
            endpointInches = getEndpointInches();

        }
        if (loopctr == 6) {
            positionInches = getPositionInches();
            inchespersec = getInchesPerSec();
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
        return mPosController.getIAccum();
    }

    public void close() {
        m_motor.close();
    }

    public double getEndpointInches() {
        return endpointInches;
    }

    public void setExtAdjust(double adj) {

        extEndAdjust = adj;
    }

    public void setWristAdjust(double adj) {

        wristAngleAdj = adj;
    }

    public boolean atTargetPosition() {
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

    public void setGoal(double goalRadians) {
        goalAngleRadians = goalRadians;
    }

    public void setControllerConstraints(TrapezoidProfile.Constraints constraints) {
        m_liftController.setConstraints(constraints);
    }

}