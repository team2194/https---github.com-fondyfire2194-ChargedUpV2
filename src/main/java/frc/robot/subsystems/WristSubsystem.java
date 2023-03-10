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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

        PICKUP_CUBE_GROUND(90),
        PICKUP_CONE_GROUND(90),

        PICKUP_CUBE_LOAD_STATION(89),
        PICKUP_CONE_LOAD_STATION(89),

        PLACE_CUBE_GROUND(60),
        PLACE_CUBE_MID_SHELF(113),
        PLACE_CUBE_TOP_SHELF(118),

        PLACE_CONE_GROUND(100),
        PLACE_CONE_MID_SHELF(120),
        PLACE_CONE_TOP_SHELF(121),

        PLACE_CONE_MID_PIPE(145),
        PLACE_CONE_TOP_PIPE(145);

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

    public final SparkMaxPIDController mVelController;


    public ArmFeedforward m_armfeedforward;

    private double inPositionBandwidth = 1;

    public boolean wristMotorConnected;

    public int faultSeen;

    private double m_positionSim;

    private double maxdegpersec = 50;

    public double radspersecpervolt = maxdegpersec / 12;

    private double positionChangeper20ms;

    public ProfiledPIDController m_wristController = new ProfiledPIDController(.01, 0, 0,
            WristConstants.wristConstraints);

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

    public WristSubsystem() {

        useSoftwareLimit = false;
        m_motor = new CANSparkMax(CanConstants.WRIST_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        mVelController = m_motor.getPIDController();
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(false);
        m_motor.setOpenLoopRampRate(1);
        m_motor.setClosedLoopRampRate(.5);

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);// vel

        mEncoder.setPositionConversionFactor(WristConstants.RADIANS_PER_ENCODER_REV);

        mEncoder.setVelocityConversionFactor(WristConstants.RADIANS_PER_ENCODER_REV / 60);

        mVelController.setOutputRange(-1, 1);

        mVelController.setFF(1 / WristConstants.MAX_RADS_PER_SEC);

        SmartDashboard.putNumber("WRDPR", WristConstants.RADIANS_PER_ENCODER_REV);

        mEncoder.setPosition(presetWristAngles.HOME.getAngleRads());

        setController(WristConstants.wristConstraints, presetWristAngles.HOME.getAngleRads(), false);

        m_motor.setSmartCurrentLimit(20);

        m_motor.setClosedLoopRampRate(.5);

        m_motor.setIdleMode(IdleMode.kBrake);

        setSoftwareLimits();

        enableSoftLimits(useSoftwareLimit);

        m_armfeedforward = new ArmFeedforward(WristConstants.ksVolts, WristConstants.kgVolts,
                WristConstants.kvWristVoltSecondsPerRadian);

       // SmartDashboard.putNumber("WRPSM", WristConstants.MAX_RADS_PER_SEC);// 1.04

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

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

        goalAngleRadians = angleRadians;
    }

    public void setController(TrapezoidProfile.Constraints constraints, double angleRads, boolean initial) {
        SmartDashboard.putNumber("GARRRRRRRRRR", angleRads);
        if (isStopped()) {
            setControllerConstraints(constraints);
            setControllerGoal(angleRads);
            goalAngleRadians = angleRads;

            if (initial)
                m_wristController.reset(new TrapezoidProfile.State(presetWristAngles.HOME.getAngleRads(), 0));
            else
                m_wristController.reset(new TrapezoidProfile.State(getAngleRadians(), 0));

        }
    }

    public void setControllerAtPosition() {
        setController(WristConstants.wristConstraints, getAngleRadians(), false);

    }

    // public double getEndpointDegrees() {

    // return endpointDegrees;
    // }

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
            return m_positionSim;

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
