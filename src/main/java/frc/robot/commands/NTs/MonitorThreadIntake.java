// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NTs;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.IntakeSubsystem;

/** Add your docs here. */
public class MonitorThreadIntake {

    private IntakeSubsystem m_intake;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable intprof = inst.getTable("intprof");

    public DoublePublisher rpm;
    public DoublePublisher cube;
    public DoublePublisher cone;
    public DoublePublisher amps;

    public MonitorThreadIntake(IntakeSubsystem intake) {

        m_intake = intake;

        rpm = intprof.getDoubleTopic("RPM").publish();
        cube = intprof.getDoubleTopic("CUBEDIST").publish();
        cone = intprof.getDoubleTopic("CONEDIST").publish();
      
        amps = intprof.getDoubleTopic("CURRENT").publish();

    }

    public void startThread() {

        // Set up thread properties and start it off
        monitorThread.setName("INT Thread");
        monitorThread.setPriority(Thread.MIN_PRIORITY);
        monitorThread.start();
    }

    Thread monitorThread = new Thread(new Runnable() {
        @Override
        public void run() {
            try {
                while (!Thread.currentThread().isInterrupted()) {
                    m_intake.tstctr++;

                    if (true) {
                        rpm.set(m_intake.rpm);
                        cube.set(m_intake.getCubeSensorDistance());
                        cone.set(m_intake.getConeSensorDistance());
                        amps.set(m_intake.amps);
                                  
                    }
                    Thread.sleep(100);
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    });

}