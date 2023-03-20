// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NTs;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    public DoublePublisher profpos;
    public DoublePublisher disterr;
    public DoublePublisher volts;
    public DoublePublisher profvel;
    public BooleanPublisher inizone;
    public DoublePublisher velerr;
    public DoublePublisher amps;
    public DoublePublisher gravval;

    public MonitorThreadExt(ExtendArmSubsystem ext) {

        m_ext = ext;

        goalinches = extprof.getDoubleTopic("GOALINCH").publish();
        velocity = extprof.getDoubleTopic("ACTVEL").publish();
        distance = extprof.getDoubleTopic("ACTDIST").publish();
        feedforward = extprof.getDoubleTopic("FFWD").publish();
        pidval = extprof.getDoubleTopic("PIDVAL").publish();
        profpos = extprof.getDoubleTopic("PROFILEPOSN").publish();
        disterr = extprof.getDoubleTopic("DISTERR").publish();
        volts = extprof.getDoubleTopic("VOLTS").publish();
        profvel = extprof.getDoubleTopic("PROFVEL").publish();
        inizone = extprof.getBooleanTopic("INIZONE").publish();
        velerr = extprof.getDoubleTopic("VELERR").publish();
        amps = extprof.getDoubleTopic("CURRENT").publish();
        gravval = extprof.getDoubleTopic("GRAVVAL").publish();

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
                        velocity.set(m_ext.inchespersec);
                        distance.set(m_ext.positionInches);
                        feedforward.set(m_ext.ff);
                        pidval.set(m_ext.pidVal);
                        volts.set(m_ext.volts);
                        profpos.set(m_ext.m_extController.getSetpoint().position);
                        disterr.set(m_ext.m_extController.getPositionError());
                        profvel.set(m_ext.m_extController.getSetpoint().velocity);
                        inizone.set(m_ext.inIZone);
                        amps.set(m_ext.amps);
                        gravval.set(m_ext.gravVal);
                        
                        
                    }
                    Thread.sleep(100);
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    });

}