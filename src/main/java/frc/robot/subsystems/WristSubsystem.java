package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Pref;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {

    /**
     * Wrist arms are defined with respect to the extend arm. positive angles are up
     * and negative down
     * Home angle position for the wrist is 90 - 51.3(mark dwg)= 38.7
     * 
     * 
     * 
     */

    public enum presetWristAngles {

        HOME(52),

        SAFE_TRAVEL(58),

        PICKUP_CUBE_GROUND(106),

        PICKUP_CONE_GROUND(105),

        PICKUP_TIPPED_CONE_GROUND(219),

        PICKUP_CUBE_LOAD_STATION(137),

        PICKUP_CONE_LOAD_STATION(165),

        PLACE_CUBE_MID_SHELF(98),

        PLACE_CUBE_TOP_SHELF(122),

        PLACE_CONE_MID_PIPE(146),

        PLACE_CONE_TOP_PIPE(163);

        private double angle;

        private presetWristAngles(double angle) {
            this.angle = angle;
        }

        public double getAngleRads() {
            return Units.degreesToRadians(this.angle);
        }

    }

    public double pidVal = 0;

    public final CANSparkMax m_motor;

    private final RelativeEncoder mEncoder;

    public ArmFeedforward m_armfeedforward;

    private double inPositionBandwidth = 1;

    public boolean wristMotorConnected;

    public int faultSeen;

    private double m_positionSim;

    private double maxdegpersec = 50;

    public double radspersecpervolt = maxdegpersec / 12;

    private double positionChangeper20ms;

    public ProfiledPIDController m_wristController = new ProfiledPIDController(0.005, 0, 0,
            WristConstants.wristConstraints,.02);

    public double deliverAngleRads;

    public double appliedOutput;

    public double amps;

    public double positionDegrees;

    public double radspersec;

    private int loopctr;

    private boolean useSoftwareLimit;

    public TrapezoidProfile.State profile;
    public double ff;

    public boolean resetFF;
    public double commandRadPerSec;
    public double goalAngleRadians;

    public int tstCtr;

    public double gravCalc;

    public double volts;

    public boolean inIZone;

    private boolean runDeliverAngle;

    public WristSubsystem() {

        useSoftwareLimit = false;
        m_motor = new CANSparkMax(CanConstants.WRIST_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(false);
        m_motor.setOpenLoopRampRate(.1);
        m_motor.setClosedLoopRampRate(.1);

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);// vel

        mEncoder.setPositionConversionFactor(WristConstants.RADIANS_PER_ENCODER_REV);

        mEncoder.setVelocityConversionFactor(WristConstants.RADIANS_PER_ENCODER_REV / 60);

        SmartDashboard.putNumber("WRDPR", WristConstants.RADIANS_PER_ENCODER_REV);

        mEncoder.setPosition(presetWristAngles.HOME.getAngleRads());

        setController(WristConstants.wristConstraints, presetWristAngles.HOME.getAngleRads(), false);

        m_motor.setSmartCurrentLimit(20);

        m_motor.setIdleMode(IdleMode.kBrake);

        setSoftwareLimits();

        enableSoftLimits(useSoftwareLimit);

        m_armfeedforward = new ArmFeedforward(WristConstants.ksVolts, WristConstants.kgVolts,
                WristConstants.kvWristVoltSecondsPerRadian, WristConstants.kaWristVoltSecondsSquaredPerRadian);

        // SmartDashboard.putNumber("WRPSM", WristConstants.MAX_RADS_PER_SEC);// 1.04

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        if (runDeliverAngle) {

            setController(WristConstants.wristConstraints, deliverAngleRads, false);

            SmartDashboard.putNumber("WDA", deliverAngleRads);
            runDeliverAngle = false;
        }

        loopctr++;

        if (faultSeen != 0)
            faultSeen = getFaults();

        if (loopctr == 5) {

            appliedOutput = getAppliedOutput();
            amps = getAmps();
            positionDegrees = getAngleDegrees();

        }

        if (loopctr == 6) {

            radspersec = getRadsPerSec();
            wristMotorConnected = checkCANOK();
            loopctr = 0;

        }

    }

    @Override
    public void simulationPeriodic() {

        positionChangeper20ms = getAppliedOutput() * radspersecpervolt / 50;

        m_positionSim += positionChangeper20ms;

    }

    public boolean checkCANOK() {
        return RobotBase.isSimulation() || m_motor.getFirmwareVersion() != 0;
    }

    public void close() {
        m_motor.close();
    }

    public void setControllerConstraints(TrapezoidProfile.Constraints constraints) {
        m_wristController.setConstraints(constraints);
    }

    public void setControllerGoal(double angleRadians) {
        m_wristController.setGoal(angleRadians);
        SmartDashboard.putNumber("GAR", m_wristController.getGoal().position);
    }

    public void setController(TrapezoidProfile.Constraints constraints, double angleRads, boolean initial) {

        if (isStopped()) {
            setControllerConstraints(constraints);
            setControllerGoal(angleRads);
            goalAngleRadians = angleRads;
            SmartDashboard.putNumber("GARRRRRRRRRR", angleRads);
            if (initial)
                m_wristController.reset(new TrapezoidProfile.State(presetWristAngles.HOME.getAngleRads(), 0));
            else
                m_wristController.reset(new TrapezoidProfile.State(getAngleRadians(), 0));

        }

        m_armfeedforward = new ArmFeedforward(Pref.getPref("wristKs"), Pref.getPref("wristKg"),
                Pref.getPref("wristKv"));

        m_wristController.setP(Pref.getPref("wristKp"));

    }

    public void setControllerAtPosition() {
        setController(WristConstants.wristConstraints, getAngleRadians(), false);
    }

    public void redoTarget() {
        setControllerGoal(m_wristController.getGoal().position);
    }


    public void runDeliverAngle(boolean on) {
        runDeliverAngle = on;
    }

    public void resetAngle() {
        mEncoder.setPosition(0);
    }

    public boolean atTargetAngle() {
        return Math.abs(goalAngleRadians - getAngleDegrees()) < inPositionBandwidth;
    }

    public double getAngleDegrees() {

        if (RobotBase.isReal())

            return Units.radiansToDegrees(mEncoder.getPosition());

        else

            return m_positionSim;

    }

    public double getAngleRadians() {

        if (RobotBase.isReal())

            return mEncoder.getPosition();

        else

            return Units.degreesToRadians(m_positionSim);

    }

    public double getAppliedOutput() {
        return m_motor.getAppliedOutput();
    }

    public boolean isStopped() {
        return Math.abs(getRadsPerSec()) < .01;
    }

    public double getAmps() {
        return m_motor.getOutputCurrent();
    }

    public double getRadsPerSec() {
        return mEncoder.getVelocity();
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

    public String getFirmware() {
        return m_motor.getFirmwareString();
    }

    public void setSoftwareLimits() {
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) WristConstants.MIN_ANGLE);
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) WristConstants.MAX_ANGLE);
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
