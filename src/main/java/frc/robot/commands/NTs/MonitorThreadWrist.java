// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NTs;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;

/** Add your docs here. */
public class MonitorThreadWrist {

    private WristSubsystem m_wrist;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable wristprof = inst.getTable("wristprof");

    public DoublePublisher goalangle;
    public DoublePublisher velocity;
    public DoublePublisher distance;
    public DoublePublisher feedforward;
    public DoublePublisher pidval;
    public DoublePublisher lastspeed;;


    public DoublePublisher profpos;
    public DoublePublisher disterr;
    public DoublePublisher volts;
    public DoublePublisher kvEst;
    public DoublePublisher profvel;
    public DoublePublisher inizone;
    public DoublePublisher velerr;

    public MonitorThreadWrist(WristSubsystem wrist) {

        m_wrist = wrist;
        goalangle = wristprof.getDoubleTopic("GOALANGLE").publish();
        velocity = wristprof.getDoubleTopic("ACTVEL").publish();
        distance = wristprof.getDoubleTopic("ACTDIST").publish();
        feedforward = wristprof.getDoubleTopic("FFWD").publish();
        pidval = wristprof.getDoubleTopic("PIDVAL").publish();
        lastspeed = wristprof.getDoubleTopic("LASTSPEED").publish();
        profpos = wristprof.getDoubleTopic("PROFILEPOSN").publish();
        disterr = wristprof.getDoubleTopic("DISTERR").publish();
        volts = wristprof.getDoubleTopic("VOLTS").publish();
        kvEst = wristprof.getDoubleTopic("KVEEST").publish();
        profvel = wristprof.getDoubleTopic("PROFVEL").publish();
        inizone = wristprof.getDoubleTopic("INIZONE").publish();
        velerr = wristprof.getDoubleTopic("VELERR").publish();

    }

    public void startThread() {

        // Set up thread properties and start it off
        monitorThread.setName("WRIST Thread");
        monitorThread.setPriority(Thread.MIN_PRIORITY);
        monitorThread.start();
    }

    Thread monitorThread = new Thread(new Runnable() {

        @Override
        public void run() {
            try {
                while (!Thread.currentThread().isInterrupted()) {
                    m_wrist.tstCtr++;

                    if (!m_wrist.isStopped()) {

                        goalangle.set(m_wrist.goalAngleRadians);
                        velocity.set(m_wrist.getRadsPerSec());
                        distance.set(m_wrist.getAngleRadians());
                        feedforward.set(m_wrist.ff);
                        pidval.set(m_wrist.pidVal);
                        volts.set(m_wrist.volts);
                        kvEst.set((m_wrist.volts - WristConstants.ksVolts - WristConstants.kgVolts)
                                / m_wrist.getRadsPerSec());
                        profpos.set(m_wrist.m_wristController.getSetpoint().position);
                        disterr.set(m_wrist.m_wristController.getPositionError());
                        velerr.set(m_wrist.m_wristController.getVelocityError());                       
                        profvel.set(m_wrist.m_wristController.getSetpoint().velocity);
                        inizone.set(m_wrist.inIZone ? -1.0 : 1.0);                  

                    }
                    Thread.sleep(100);
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    });

}