package frc.robot.subsystems;

import java.util.Map;

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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

        START_ANGLE(120),

        SAFE_TRAVEL(120),

        PICKUP_CUBE_GROUND(90),
        PICKUP_CONE_GROUND(90),

        PICKUP_CUBE_LOAD_STATION(-3),
        PICKUP_CONE_LOAD_STATION(-3),

        PLACE_CUBE_GROUND(00),
        PLACE_CUBE_MID_SHELF(17),
        PLACE_CUBE_TOP_SHELF(18),

        PLACE_CONE_GROUND(10),
        PLACE_CONE_MID_SHELF(20),
        PLACE_CONE_TOP_SHELF(21),

        PLACE_CONE_MID_PIPE(-45),
        PLACE_CONE_TOP_PIPE(-45);

        private double angle;

        private presetWristAngles(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return Units.degreesToRadians(this.angle);
        }

    }

    public final CANSparkMax m_motor;
    private final RelativeEncoder mEncoder;
    public final SparkMaxPIDController mPosController;

    public ArmFeedforward m_armfeedforward;

    private double inPositionBandwidth = 1;

    public boolean wristMotorConnected;

    public int faultSeen;

    public double endpointDegrees;

    private double m_positionSim;

    private double maxdegpersec = 50;

    public double degreespersecpervolt = maxdegpersec / 12;

    private double positionChangeper20ms;

    public ProfiledPIDController m_wristController = new ProfiledPIDController(.05, 0, 0,
            WristConstants.wristConstraints);

    public double deliverAngle;

    public double appliedOutput;

    public double amps;

    public double positionDegrees;

    public double degreespersec;

    private int loopctr;

    private boolean useSoftwareLimit;

    public TrapezoidProfile.State profile;
    public double ff;

    public boolean resetFF;
    public double commandDPS;
    public double goalAngleRadians;

    public WristSubsystem() {

        useSoftwareLimit = false;
        m_motor = new CANSparkMax(CanConstants.WRIST_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        mPosController = m_motor.getPIDController();
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(true);
        m_motor.setOpenLoopRampRate(1);
        m_motor.setClosedLoopRampRate(1);

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);// vel

        mEncoder.setPositionConversionFactor(WristConstants.RADIANS_PER_ENCODER_REV);

        mEncoder.setVelocityConversionFactor(WristConstants.RADIANS_PER_ENCODER_REV / 60);

        SmartDashboard.putNumber("WRDPR", WristConstants.RADIANS_PER_ENCODER_REV);

        mPosController.setOutputRange(-.5, .5);

        mPosController.setP(.001);

        mEncoder.setPosition(presetWristAngles.START_ANGLE.getAngle());

        setGoal(mEncoder.getPosition());

        m_motor.setSmartCurrentLimit(20);

        m_motor.setClosedLoopRampRate(.5);

        m_motor.setIdleMode(IdleMode.kBrake);

        setSoftwareLimits();

        enableSoftLimits(useSoftwareLimit);

        if (RobotBase.isSimulation()) {

            m_positionSim = WristConstants.MIN_ANGLE;

            mPosController.setP(5);
        }


    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        loopctr++;
        if (RobotBase.isReal() && DriverStation.isDisabled()

                && endpointDegrees != getAngleDegrees())

            endpointDegrees = getAngleDegrees();

        if (faultSeen != 0)
            faultSeen = getFaults();

        if (loopctr == 5) {

            appliedOutput = getAppliedOutput();
            amps = getAmps();
            positionDegrees = getAngleDegrees();

        }

        if (loopctr == 6) {

            degreespersec = getDegreesPerSec();
            wristMotorConnected = checkCANOK();
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

    public void setControllerConstraints(TrapezoidProfile.Constraints constraints) {
        m_wristController.setConstraints(constraints);
    }

    public void setGoal(double angleRadians) {

        goalAngleRadians = angleRadians;
    }

    // public double getEndpointDegrees() {

    // return endpointDegrees;
    // }

    public void resetAngle() {

        mEncoder.setPosition(0);

        endpointDegrees = 0;

    }

    public void startPosition() {
        setGoal(Pref.getPref("WRISTDEG"));

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
        return Math.abs(mEncoder.getVelocity()) < .05;
    }

    public double getAmps() {
        return m_motor.getOutputCurrent();
    }

    public double getDegreesPerSec() {
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
