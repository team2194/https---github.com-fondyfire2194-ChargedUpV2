// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NTs;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.subsystems.ExtendArmSubsystem;

/** Add your docs here. */
public class MonitorThreadExt {

    private ExtendArmSubsystem m_ext;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable extprof = inst.getTable("extprof1");

    public DoublePublisher goalinches;
    public DoublePublisher velocity;
    public DoublePublisher distance;
    public DoublePublisher feedforward;
    public DoublePublisher pidval;
    public DoublePublisher lastspeed;;
    public DoublePublisher accel;
    public DoublePublisher profpos;
    public DoublePublisher disterr;
    public DoublePublisher volts;
    public DoublePublisher kvEst;
    public DoublePublisher profvel;
    public DoublePublisher inizone;
    public DoublePublisher velerr;

    public MonitorThreadExt(ExtendArmSubsystem ext) {

        m_ext = ext;

        goalinches = extprof.getDoubleTopic("GOALINCH").publish();
        velocity = extprof.getDoubleTopic("ACTVEL").publish();
        distance = extprof.getDoubleTopic("ACTDIST").publish();
        feedforward = extprof.getDoubleTopic("FFWD").publish();
        pidval = extprof.getDoubleTopic("PIDVAL").publish();
        lastspeed = extprof.getDoubleTopic("LASTSPEED").publish();
        accel = extprof.getDoubleTopic("ACCEL").publish();
        profpos = extprof.getDoubleTopic("PROFILEPOSN").publish();
        disterr = extprof.getDoubleTopic("DISTERR").publish();
        volts = extprof.getDoubleTopic("VOLTS").publish();
        kvEst = extprof.getDoubleTopic("KVEEST").publish();
        profvel = extprof.getDoubleTopic("PROFVEL").publish();
        inizone = extprof.getDoubleTopic("INIZONE").publish();
        velerr = extprof.getDoubleTopic("VELERR").publish();

    }

    public void startThread() {

        // Set up thread properties and start it off
        monitorThread.setName("EXT Thread");
        monitorThread.setPriority(Thread.MIN_PRIORITY);
        monitorThread.start();
    }

    Thread monitorThread = new Thread(new Runnable() {
        @Override
        public void run() {
            try {
                while (!Thread.currentThread().isInterrupted()) {
                    m_ext.tstctr++;

                    if (!m_ext.isStopped()) {
                        goalinches.set(m_ext.goalInches);
                        velocity.set(m_ext.getInchesPerSec());
                        distance.set(m_ext.getPositionInches());
                        feedforward.set(m_ext.ff);
                        pidval.set(m_ext.pidVal);
                        volts.set(m_ext.volts);
                        kvEst.set((m_ext.getAppliedOutput() - ExtendArmConstants.ksExtArmVolts)
                                / m_ext.getInchesPerSec());
                        profpos.set(m_ext.m_extController.getSetpoint().position);
                        disterr.set(m_ext.m_extController.getPositionError());
                        profvel.set(m_ext.m_extController.getSetpoint().velocity);
                        inizone.set(m_ext.inIZone ? -1.0 : 1.0);                  
                    }
                    Thread.sleep(100);
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    });

}