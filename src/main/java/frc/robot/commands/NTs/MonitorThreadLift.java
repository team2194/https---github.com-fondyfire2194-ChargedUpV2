// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NTs;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.subsystems.LiftArmSubsystem;

/** Add your docs here. */
public class MonitorThreadLift {

    private LiftArmSubsystem m_lift;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable liftprof = inst.getTable("liftprof");

    public DoublePublisher goalAngle;
    public DoublePublisher velocity;
    public DoublePublisher feedforward;
    public DoublePublisher pidval;
    public DoublePublisher lastspeed;;
    public DoublePublisher accel;

    public DoublePublisher profpos;
    public DoublePublisher disterr;
    public DoublePublisher volts;
    public DoublePublisher kvEst;
    public DoublePublisher profvel;

    public MonitorThreadLift(LiftArmSubsystem lift) {

        m_lift = lift;

        goalAngle = liftprof.getDoubleTopic("GOALANGLERADS").publish();
        velocity = liftprof.getDoubleTopic("ACTVEL").publish();
        feedforward = liftprof.getDoubleTopic("FFWD").publish();
        pidval = liftprof.getDoubleTopic("PIDVAL").publish();
        lastspeed = liftprof.getDoubleTopic("LASTSPEED").publish();
        accel = liftprof.getDoubleTopic("ACCEL").publish();
        profpos = liftprof.getDoubleTopic("PROFILEPOSN").publish();
        disterr = liftprof.getDoubleTopic("DISTERR").publish();
        volts = liftprof.getDoubleTopic("VOLTS").publish();
        kvEst = liftprof.getDoubleTopic("KVEEST").publish();
        profvel = liftprof.getDoubleTopic("PROFVEL").publish();

    }

    public void startThread() {

        // Set up thread properties and start it off
        monitorThread.setName("LIFT Thread");
        monitorThread.setPriority(Thread.MIN_PRIORITY);
        monitorThread.start();
    }

    Thread monitorThread = new Thread(new Runnable() {
        @Override
        public void run() {
            try {
                while (!Thread.currentThread().isInterrupted()) {
                    m_lift.tstCtr++;

                    if (true) {

                        goalAngle.set(m_lift.goalAngleRadians);
                        velocity.set(m_lift.getCanCoderRateRadsPerSec());
                        feedforward.set(m_lift.ff);
                        pidval.set(m_lift.m_liftController.getSetpoint().position);
                        accel.set(m_lift.positionRadians);
                        lastspeed.set(m_lift.m_liftController.getSetpoint().velocity);
                        volts.set(m_lift.volts);
                        kvEst.set((m_lift.volts - LiftArmConstants.ksVolts - LiftArmConstants.kGVolts)
                                / m_lift.getCanCoderRateRadsPerSec());
                        profpos.set(m_lift.m_liftController.getSetpoint().position);
                        disterr.set(m_lift.m_liftController.getPositionError());
                        profvel.set(m_lift.m_liftController.getSetpoint().velocity);

                    }
                    Thread.sleep(100);
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    });

}